import rclpy
from rclpy.node import Node
import json, math, sys, time, os, select, termios, atexit, signal
from textwrap import dedent
from typing import Optional, List, Tuple

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from .a_star import AStarOnOccGrid

# -------------------- 日志小工具 --------------------
def log_block(logger, text: str):
    for line in dedent(text).strip("\n").splitlines():
        logger.info(line)

# -------------------- 工具 --------------------
def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class TTYGuard:
    """让 stdin 在节点生命周期内保持 raw+noecho，退出恢复"""
    def __init__(self):
        self.fd = None
        self.old = None
        self.enabled = False

    def enable(self) -> bool:
        if not sys.stdin.isatty():
            return False
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        new = termios.tcgetattr(self.fd)
        # 输入模式：关掉特殊翻译
        new[0] &= ~(termios.BRKINT | termios.ICRNL | termios.INPCK | termios.ISTRIP | termios.IXON)
        # 输出模式：保持 OPOST 开着，避免日志错位
        # new[1] &= ~(termios.OPOST)   # 不改
        # 本地模式：关回显、规范模式、扩展处理；保留信号(Ctrl+C)
        new[3] &= ~(termios.ECHO | termios.ICANON | termios.IEXTEN)
        # 最小读取 & 超时
        new[6][termios.VMIN]  = 0
        new[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSADRAIN, new)
        self.enabled = True
        return True

    def restore(self):
        if self.enabled and self.fd is not None and self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            self.enabled = False

def read_key_nonblocking(timeout_sec: float = 0.02) -> Optional[str]:
    """
    非阻塞读键。支持普通键、CSI 箭头(\x1b[A/B/C/D) 和 SS3 箭头(\x1bOA/OB/OC/OD)。
    """
    if not sys.stdin.isatty():
        return None
    fd = sys.stdin.fileno()
    r, _, _ = select.select([fd], [], [], timeout_sec)
    if not r:
        return None

    ch1 = os.read(fd, 1).decode(errors='ignore')
    if not ch1:
        return None

    if ch1 != '\x1b':  # 普通键
        return ch1

    # 第二字节
    r, _, _ = select.select([fd], [], [], 0.03)
    if not r:
        return ch1
    ch2 = os.read(fd, 1).decode(errors='ignore')

    # 第三字节
    r, _, _ = select.select([fd], [], [], 0.03)
    if not r:
        return ch1 + ch2
    ch3 = os.read(fd, 1).decode(errors='ignore')

    # 规范化成 '\x1b[<X>'
    if ch2 == '[' and ch3 in 'ABCD':
        return f'\x1b[{ch3}'
    if ch2 == 'O' and ch3 in 'ABCD':
        return f'\x1b[{ch3}'
    return ch1 + ch2 + ch3

# -------------------- 主节点 --------------------
class AStarNode(Node):
    def __init__(self):
        super().__init__('a_star_node')

        # —— 参数 ——
        self.declare_parameter('map_topic', '/rtabmap/grid_prob_map')
        self.declare_parameter('odom_topic', '/utlidar/robot_odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('replan_period', 0.5)
        self.declare_parameter('occ_threshold', 50)
        self.declare_parameter('inflation_radius', 0.3)
        self.declare_parameter('allow_unknown', True)
        self.declare_parameter('enable_simplify', False)

        # 键盘 & 模式
        self.declare_parameter('keyboard_enabled', True)
        self.declare_parameter('keyboard_period', 0.05)
        self.declare_parameter('start_mode', 'manual')     # 默认手动
        self.declare_parameter('manual_hold_timeout', 0.1) # 键盘松手后保持时间

        # 固定速度（键盘直控）
        self.declare_parameter('manual_speed', 0.4)     # x
        self.declare_parameter('manual_side',  0.3)     # y
        self.declare_parameter('manual_turn',  0.8)      # yaw(z)

        # 读取参数
        self.map_topic = self.get_parameter('map_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.period = float(self.get_parameter('replan_period').value)

        self.kb_enabled = bool(self.get_parameter('keyboard_enabled').value)
        self.kb_period = float(self.get_parameter('keyboard_period').value)
        self.manual_speed = float(self.get_parameter('manual_speed').value)
        self.manual_side  = float(self.get_parameter('manual_side').value)
        self.manual_turn  = float(self.get_parameter('manual_turn').value)
        self.manual_hold_timeout = float(self.get_parameter('manual_hold_timeout').value)
        self.manual_mode = (str(self.get_parameter('start_mode').value).lower() == 'manual')

        # 规划器
        self.planner = AStarOnOccGrid(
            occ_threshold=int(self.get_parameter('occ_threshold').value),
            inflation_radius_m=float(self.get_parameter('inflation_radius').value),
            allow_unknown=bool(self.get_parameter('allow_unknown').value),
            enable_simplify=bool(self.get_parameter('enable_simplify').value),
        )

        # —— 订阅/发布 ——
        self.create_subscription(OccupancyGrid, self.map_topic,  self.map_cb,  5)
        self.create_subscription(Odometry,      self.odom_topic, self.odom_cb, 10)
        self.create_subscription(PoseStamped,   self.goal_topic, self.goal_cb, 10)

        # Unitree Request 发布
        from unitree_api.msg import Request
        self.RequestMsg = Request
        self.control_publisher = self.create_publisher(self.RequestMsg, '/api/sport/request', 10)

        # 可视化
        self.pub_path      = self.create_publisher(Path,   '/a_star_path',            10)
        self.pub_mkr_path  = self.create_publisher(Marker, '/a_star_markers/path',     1)
        self.pub_mkr_start = self.create_publisher(Marker, '/a_star_markers/start',    1)
        self.pub_mkr_goal  = self.create_publisher(Marker, '/a_star_markers/goal',     1)

        # 缓存
        self.map: Optional[OccupancyGrid] = None
        self.odom: Optional[Odometry] = None
        self.goal: Optional[PoseStamped] = None

        # 手动状态（键盘直控）
        self.kb_cmd = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.kb_last_time = 0.0

        # TTY raw 模式守护
        self.tty = TTYGuard()
        kb_ok = False
        if self.kb_enabled:
            kb_ok = self.tty.enable()
            atexit.register(self.tty.restore)
        self.get_logger().info(f"STDIN is TTY: {sys.stdin.isatty()}, keyboard_raw_enabled: {kb_ok}")

        # 定时器：主控制 + （可选）键盘小定时器
        self.timer = self.create_timer(self.period, self.timer_cb)
        if self.kb_enabled and kb_ok:
            self.kb_timer = self.create_timer(self.kb_period, self.keyboard_tick)
            self._print_keyboard_help()
        else:
            self.kb_timer = None
            if self.kb_enabled and not kb_ok:
                self.get_logger().warn("键盘读取已启用但未获得 TTY raw 模式。")

        log_block(self.get_logger(), f"""
        A* node started.
        map: {self.map_topic}
        odom: {self.odom_topic}
        goal: {self.goal_topic}
        period: {self.period}s
        mode: {'manual' if self.manual_mode else 'auto'}
        """)

    # —— 订阅回调 ——
    def map_cb(self, msg: OccupancyGrid): self.map = msg
    def odom_cb(self, msg: Odometry): self.odom = msg
    def goal_cb(self, msg: PoseStamped): self.goal = msg

    # —— 键盘定时器：高频读取 & 更新手动命令 ——
    def keyboard_tick(self):
        # 读取若干键（高频短超时）
        for _ in range(4):
            k = read_key_nonblocking(0.005)
            if k is None:
                break
            self._handle_key(k)

        # 手动模式下，无输入太久自动停（仅键盘通道）
        if self.manual_mode and (time.time() - self.kb_last_time > self.manual_hold_timeout):
            if any(abs(v) > 1e-6 for v in self.kb_cmd.values()):
                self.kb_cmd = {"x": 0.0, "y": 0.0, "z": 0.0}

    def _handle_key(self, k: str):
        k = k.lower()
        # 模式切换
        if k in ('m', 'q'):
            self.manual_mode = not self.manual_mode
            self._publish_stop()
            self.get_logger().info(f"切换到 {'手动' if self.manual_mode else '自动(A*)'} 模式")
            return

        if not self.manual_mode:
            return

        # 手动模式：键盘固定速度
        vx, vy, wz = self.kb_cmd["x"], self.kb_cmd["y"], self.kb_cmd["z"]
        if k == ' ':
            vx, vy, wz = 0.0, 0.0, 0.0
        elif k in ('w'):         # 前 
            vx, vy, wz = self.manual_speed, 0.0, 0.0
        elif k in ('s'):         # 后 
            vx, vy, wz = -self.manual_speed, 0.0, 0.0
        elif k in ('a',):                  # 左移
            vx, vy, wz = 0.0, 0.0, +self.manual_turn
        elif k in ('d',):                  # 右移
            vx, vy, wz = 0.0, 0.0, -self.manual_turn
        elif k in ('z',):                  # 左移
            vx, vy, wz = 0.0, self.manual_speed, 0.0
        elif k in ('c',):                  # 右移
            vx, vy, wz = 0.0, -self.manual_speed, 0.0
        else:
            return

        self.kb_cmd = {"x": float(vx), "y": float(vy), "z": float(wz)}
        self.kb_last_time = time.time()

    def _downsample_path(self, path_xy: List[Tuple[float, float]], step: int = 4) -> List[Tuple[float, float]]:
        """
        对 A* 的路径做简单抽点：每 step 个点取一个，保留首尾。
        step=3 表示大约每 3 个栅格保留 1 个点。
        """
        if len(path_xy) <= 2 or step <= 1:
            return path_xy

        new_path = [path_xy[0]]
        for i in range(step, len(path_xy) - 1, step):
            new_path.append(path_xy[i])
        new_path.append(path_xy[-1])
        return new_path

    # —— 主定时器：模式分发 + 控制下发 ——
    def timer_cb(self):
       # 手动模式：只在非零时发布；全 0 时什么都不发
        if self.manual_mode:
            cmd = dict(self.kb_cmd)
            if any(abs(v) > 1e-6 for v in cmd.values()):
                self._publish_request(cmd)
            return

        # 自动模式：A* 规划 + 跟随
        if self.odom is None or self.map is None or self.goal is None:
            return

        sx = float(self.odom.pose.pose.position.x)
        sy = float(self.odom.pose.pose.position.y)
        gx = float(self.goal.pose.position.x)
        gy = float(self.goal.pose.position.y)

        path_xy: Optional[List[Tuple[float, float]]] = self.planner.plan_on_grid(
            self.map, (sx, sy), (gx, gy)
        )
        if not path_xy:
            self.get_logger().warn("A*: no path.")
            return
        
        # ★ 抽点：把 path 稍微稀疏一点
        path_xy = self._downsample_path(path_xy, step=8)
        self.get_logger().info(f"A* raw path_xy: {path_xy}")

        # 发布 Path
        frame = self.map.header.frame_id or self.goal.header.frame_id or "map"
        path_msg = Path()
        path_msg.header.frame_id = frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path_xy:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.pub_path.publish(path_msg)
        self.publish_markers(frame, path_xy, (sx, sy), (gx, gy))
        self.get_logger().info(f"A*: path poses={len(path_msg.poses)}")
        self.get_logger().info(f"qidian={(sx, sy)}, zhongdian={(gx, gy)}")
        

        # 到达判定
        if math.hypot(gx - sx, gy - sy) <= 0.2:
            self._publish_stop()
            self.get_logger().info("机器狗已经到达终点")
            return

        x1, y1 = path_xy[1]  # 取下一个目标点

        # 控制律
        q = self.odom.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        dx, dy = x1 - sx, y1 - sy
        target_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
        dist_err = math.hypot(dx, dy)

        vx_body =  dx * math.cos(yaw) + dy * math.sin(yaw)
        vy_body = -dx * math.sin(yaw) + dy * math.cos(yaw)

        max_vx, min_vx = 0.3, 0.15
        max_vy, max_wz = 0.15, 0.8
        large_yaw_thr = 0.4

        if dist_err < 0.2:
            self._publish_stop()
            return

        if abs(yaw_err) > large_yaw_thr:
            vx, vy, wz = 0.0, 0.0, max(-max_wz, min(max_wz, 1.5 * yaw_err))
        else:
            vx = vx_body * 0.8
            vx = max(min_vx, min(max_vx, vx)) if vx > 0 else max(-max_vx, min(-min_vx, vx))
            vy = max(-max_vy, min(max_vy, vy_body * 0.5))
            wz = max(-max_wz, min(max_wz, 1.2 * yaw_err))

        self._publish_request({"x": float(vx), "y": float(vy), "z": float(wz)})

    # —— 可视化 ——
    def publish_markers(self, frame: str,
                        path_xy: List[Tuple[float, float]],
                        start_xy: Tuple[float, float],
                        goal_xy: Tuple[float, float]):
        m_path = Marker()
        m_path.header.frame_id = frame
        m_path.header.stamp = self.get_clock().now().to_msg()
        m_path.ns = 'a_star'; m_path.id = 1
        m_path.type = Marker.LINE_STRIP
        m_path.action = Marker.ADD
        m_path.scale.x = 0.06
        m_path.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        m_path.points = [Point(x=float(x), y=float(y), z=0.0) for (x, y) in path_xy]
        self.pub_mkr_path.publish(m_path)

        m_start = Marker()
        m_start.header = m_path.header
        m_start.ns = 'a_star'; m_start.id = 2
        m_start.type = Marker.SPHERE
        m_start.action = Marker.ADD
        m_start.scale.x = m_start.scale.y = m_start.scale.z = 0.18
        m_start.color = ColorRGBA(r=0.0, g=0.6, b=1.0, a=1.0)
        m_start.pose.position.x = float(start_xy[0]); m_start.pose.position.y = float(start_xy[1])
        self.pub_mkr_start.publish(m_start)

        m_goal = Marker()
        m_goal.header = m_path.header
        m_goal.ns = 'a_star'; m_goal.id = 3
        m_goal.type = Marker.SPHERE
        m_goal.action = Marker.ADD
        m_goal.scale.x = m_goal.scale.y = m_goal.scale.z = 0.2
        m_goal.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
        m_goal.pose.position.x = float(goal_xy[0]); m_goal.pose.position.y = float(goal_xy[1])
        self.pub_mkr_goal.publish(m_goal)

    # —— Unitree Request 下发 ——
    def _publish_request(self, move_cmd: dict):
        req = self.RequestMsg()
        req.header.identity.api_id = 1008
        req.parameter = json.dumps(move_cmd, separators=(',', ':'))
        self.control_publisher.publish(req)

    def _publish_stop(self):
        self.kb_cmd = {"x": 0.0, "y": 0.0, "z": 0.0}

    def _print_keyboard_help(self):
        log_block(self.get_logger(), """
        === 键盘控制 ===
        m 或 q : 切换 手动 <-> 自动(A*)
        手动模式下：
          W/S        : 前进/后退（固定速度）
          Z/C        : 左/右 侧移
          A/D        : 左/右 转向（角速度）
          Space      : 急停
        ====================
        """)

def main():
    rclpy.init()
    node = AStarNode()

    # 优雅退出：Ctrl+C 或 SIGTERM
    shutting_down = False
    def _shutdown_handler(*_):
        nonlocal shutting_down
        if shutting_down:
            return
        shutting_down = True
        try: node.get_logger().info("正在停止节点…")
        except Exception: pass
        try: node.tty.restore()
        except Exception: pass
        try: node.destroy_node()
        except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass

    signal.signal(signal.SIGINT, _shutdown_handler)
    signal.signal(signal.SIGTERM, _shutdown_handler)

    try:
        rclpy.spin(node)
    finally:
        _shutdown_handler()

if __name__ == '__main__':
    main()
