
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge  # 用于将ROS图像消息转换为OpenCV格式图像
import numpy as np
import json
import math
from sensor_msgs.msg import Image as SensorImage  # 使用别名避免与PIL.Image冲突
from unitree_api.msg import Request
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import make_interp_spline
from typing import Optional, List, Tuple
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tf_transformations import euler_from_quaternion
from .a_star import AStarOnOccGrid

class AStarNode(Node):
    def __init__(self):
        super().__init__('a_star_node')
        self.odom_subscription = self.create_subscription(   
            PoseStamped,
            '/utlidar/robot_pose',
            self.odom_callback,
            10
        )
        self.control_publisher = self.create_publisher(Request, '/api/sport/request', 10)
        # 参数（都可 --ros-args -p xxx:=yyy 覆盖）
        self.declare_parameter('map_topic', '/rtabmap/grid_prob_map')
        self.declare_parameter('odom_topic', '/utlidar/robot_odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('replan_period', 0.5)            # s
        self.declare_parameter('occ_threshold', 70)            # >=50 视为障碍
        self.declare_parameter('inflation_radius', 0.)         # m
        self.declare_parameter('allow_unknown', True)            # 未知(-1)当可通行

        self.map_topic = self.get_parameter('map_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.period = float(self.get_parameter('replan_period').value)

        self.planner = AStarOnOccGrid(
            occ_threshold=int(self.get_parameter('occ_threshold').value),
            inflation_radius_m=float(self.get_parameter('inflation_radius').value),
            allow_unknown=bool(self.get_parameter('allow_unknown').value),
        )

        # 缓存
        self.map: Optional[OccupancyGrid] = None
        self.odom: Optional[Odometry] = None
        self.goal: Optional[PoseStamped] = None

        # 订阅
        self.create_subscription(OccupancyGrid, self.map_topic,  self.map_cb,  5)
        self.create_subscription(Odometry,      self.odom_topic, self.odom_cb, 10)
        self.create_subscription(PoseStamped,   self.goal_topic, self.goal_cb, 10)

        # 发布
        self.pub_path = self.create_publisher(Path, '/a_star_path', 10)
        self.pub_mkr_path  = self.create_publisher(Marker, '/a_star_markers/path', 1)
        self.pub_mkr_start = self.create_publisher(Marker, '/a_star_markers/start', 1)
        self.pub_mkr_goal  = self.create_publisher(Marker, '/a_star_markers/goal', 1)
        self.pub_mkr_obs   = self.create_publisher(Marker, '/a_star_markers/obstacles', 1)  # 可选：此处暂不画障碍点，保持接口

        # 定时重规划
        self.timer = self.create_timer(self.period, self.timer_cb)
        self.get_logger().info(
            f"A* node started. map:{self.map_topic} odom:{self.odom_topic} goal:{self.goal_topic} period:{self.period}s"
        )

    def odom_callback(self, msg):
        """
        处理 /unitree_go2/odom 话题消息的回调函数，这里简单打印消息内容示例
        你可以根据实际需求进一步解析和处理具体的字段信息，比如位置、姿态等
        """
        self.position = (msg.pose.position.x, msg.pose.position.y)
        self.orientation=(msg.pose.orientation.w,msg.pose.orientation.x
                          ,msg.pose.orientation.y,msg.pose.orientation.z)
        
    # === 回调：只缓存最新消息 ===
    def map_cb(self, msg: OccupancyGrid):
        self.map = msg

    def odom_cb(self, msg: Odometry):
        self.odom = msg

    def goal_cb(self, msg: PoseStamped):
        self.goal = msg

    # === 定时器重规划 ===
    def timer_cb(self):
        if self.map is None or self.odom is None or self.goal is None:
            return

        # 起点（用 odom 中的位置；假设 map/goal 与 odom 同 frame 或已对齐到 map）
        sx = self.odom.pose.pose.position.x
        sy = self.odom.pose.pose.position.y

        # 终点（来自 /goal）
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y

        # 计划
        path_xy: Optional[List[Tuple[float, float]]] = self.planner.plan_on_grid(
            self.map, (sx, sy), (gx, gy)
        )
        if not path_xy:
            self.get_logger().warn("A*: no path.")
            return

        # 发布 Path（用 map 或 goal 的 frame；优先 goal 的 header.frame_id）
        frame = self.goal.header.frame_id or self.map.header.frame_id or "map"
        path = Path()
        path.header.frame_id = frame
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in path_xy:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.pub_path.publish(path)
        self.publish_markers(frame, path_xy, (sx, sy), (gx, gy))
        self.get_logger().info(f"A*: path poses={len(path.poses)}")

        
        orientation_w,orientation_x,orientation_y,orientation_z=self.orientation  #取出旋转信息 通过这个四元数组可以计算出朝向角yaw
        
        # 根据旋转信息的四元数组计算偏航角 (yaw) yaw=0时朝向x轴正方向
        siny_cosp = 2 * (orientation_w * orientation_z + orientation_x * orientation_y)
        cosy_cosp = 1 - 2 * (orientation_y * orientation_y + orientation_z * orientation_z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if math.dist((gx,gy),(sx, sy))>0.2:

            if len(path_xy) >= 2:   # 取路径上的第二个点作为目标，避免原地打转
                x1, y1 = path_xy[1]
            else:
                x1, y1 = path_xy[0]

            dx=x1-sx
            dy=y1-sy
            target_yaw = math.atan2(y1 - sy, x1 - sx)

            vx = dx * math.cos(current_yaw) + dy * math.sin(current_yaw)
            vy = -dx * math.sin(current_yaw) + dy * math.cos(current_yaw)
            vyaw=target_yaw-current_yaw

            if vyaw>0.3 or vyaw<-0.3:
                vx=0
                vy=0
                vyaw=max(-0.8,min(vyaw,0.8))
                move_cmd = {
                    "x": float(vx),
                    "y": float(vy), 
                    "z": float(vyaw)  # 注意：官方使用"z"表示yaw速度
                }
            else:
                vx=max(-0.5,min(vx,0.5))  #限制速度不太大不太小
                if vx > 0:
                    vx = max(0.2, vx)  # 正向时不低于 0.2
                elif vx < 0:
                    vx = min(-0.2, vx)  # 负向时不高于 -0.2
                vy=max(-0.1,min(vy,0.1))
                vyaw=max(-0.8,min(vyaw,0.8))
                move_cmd = {
                    "x": float(vx),
                    "y": float(vy), 
                    "z": float(vyaw)  # 注意：官方使用"z"表示yaw速度
                }
            # 创建 Request 消息
            req = Request()
            # 设置 API ID
            req.header.identity.api_id = 1008  # ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW
            req.parameter = json.dumps(move_cmd, separators=(',', ':'))
            # 发布 Request 消息
            self.control_publisher.publish(req)
            self.get_logger().info(f"机器狗")
        else:
            self.get_logger().info(f"机器狗已经到达终点")

    # === RViz 可视化 ===
    def publish_markers(self, frame: str,
                        path_xy: List[Tuple[float, float]],
                        start_xy: Tuple[float, float],
                        goal_xy: Tuple[float, float]):

        # 路径线
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

        # 起点
        m_start = Marker()
        m_start.header = m_path.header
        m_start.ns = 'a_star'; m_start.id = 2
        m_start.type = Marker.SPHERE
        m_start.action = Marker.ADD
        m_start.scale.x = m_start.scale.y = m_start.scale.z = 0.18
        m_start.color = ColorRGBA(r=0.0, g=0.6, b=1.0, a=1.0)
        m_start.pose.position.x = float(start_xy[0])
        m_start.pose.position.y = float(start_xy[1])
        self.pub_mkr_start.publish(m_start)

        # 终点
        m_goal = Marker()
        m_goal.header = m_path.header
        m_goal.ns = 'a_star'; m_goal.id = 3
        m_goal.type = Marker.SPHERE
        m_goal.action = Marker.ADD
        m_goal.scale.x = m_goal.scale.y = m_goal.scale.z = 0.2
        m_goal.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
        m_goal.pose.position.x = float(goal_xy[0])
        m_goal.pose.position.y = float(goal_xy[1])
        self.pub_mkr_goal.publish(m_goal)


def main():
    rclpy.init()
    node = AStarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()











# class VisionObstacleDetectionNode(Node):
#     def __init__(self, node_name):
#         super().__init__(node_name)
#         self.gx,self.gy=10,5#输入终点坐标 gx正值代表机器狗正前方，gy正值代表机器狗正左方
#         self.sign=0
#         self.obstacle_x,self.obstacle_y=[],[]
#         self.yolo=YOLO()
#         self.position = None # 用来存储机器人位置   
#         # 创建发布者，用于发布控制指令
#         self.control_publisher = self.create_publisher(Request, '/api/sport/request', 10)


#         self.odom_subscription = self.create_subscription(   
#             PoseStamped,
#             '/utlidar/robot_pose',
#             self.odom_callback,
#             10
#         )
#         self.odom_subscription  # 防止未使用变量警告

#         # 创建定时器，每秒调用一次 timer_callback
#         self.timer = self.create_timer(0.2, self.timer_callback)

#     def odom_callback(self, msg):
#         """
#         处理 /unitree_go2/odom 话题消息的回调函数，这里简单打印消息内容示例
#         你可以根据实际需求进一步解析和处理具体的字段信息，比如位置、姿态等
#         """
#         self.position = (msg.pose.position.x, msg.pose.position.y)
#         self.orientation=(msg.pose.orientation.w,msg.pose.orientation.x
#                           ,msg.pose.orientation.y,msg.pose.orientation.z)



#     def timer_callback(self):
#         """
#         定时器回调函数，每秒执行一次
#         """

        
#         position_x, position_y = self.position  #取出机器狗的当前位置信息
        
#         orientation_w,orientation_x,orientation_y,orientation_z=self.orientation  #取出旋转信息 通过这个四元数组可以计算出朝向角yaw
        
#         # 根据旋转信息的四元数组计算偏航角 (yaw) yaw=0时朝向x轴正方向
#         siny_cosp = 2 * (orientation_w * orientation_z + orientation_x * orientation_y)
#         cosy_cosp = 1 - 2 * (orientation_y * orientation_y + orientation_z * orientation_z)
#         current_yaw = math.atan2(siny_cosp, cosy_cosp)

#         if self.sign==0:
#             ttx=position_x+self.gx*math.cos(current_yaw)-self.gy*math.sin(current_yaw)
#             tty=position_y+self.gx*math.sin(current_yaw)+self.gy*math.cos(current_yaw)
#             self.gx=ttx
#             self.gy=tty
#             self.get_logger().info(f"gx={self.gx},gy={self.gy},yaw={current_yaw}")
#             self.sign=1
#         self.get_logger().info(f"机器狗当前位置={position_x},{position_y},终点位置={self.gx},{self.gy}")  #打印机器狗当前位置
#         path = planner()
#         path_x, path_y ,obstacle_x,obstacle_y= path.path_planner(position_x, position_y, coordinate,current_yaw,self.gx,self.gy,self.obstacle_x,self.obstacle_y)  # 传入机器狗位置和检测框坐标，终点坐标
#         path_x.reverse()
#         path_y.reverse()
#         path_x[0]=position_x
#         path_y[0]=position_y
#         self.get_logger().info(f"path_x={path_x},path_y={path_y}")
#         if len(path_x) == 1:  #防止终点被占用规划不出路径报错
#             path_x.extend([path_x[0]] * 2) 
#             path_y.extend([path_y[0]] * 2)

#         if math.dist((self.gx,self.gy),(position_x, position_y))>0.2:
#             dx=path_x[1]-position_x
#             dy=path_y[1]-position_y
#             target_yaw = math.atan2(path_y[1] - position_y, path_x[1] - position_x)

#             vx = dx * math.cos(current_yaw) + dy * math.sin(current_yaw)
#             vy = -dx * math.sin(current_yaw) + dy * math.cos(current_yaw)
#             vyaw=target_yaw-current_yaw

#             if vyaw>0.3 or vyaw<-0.3:
#                 vx=0
#                 vy=0
#                 vyaw=max(-0.8,min(vyaw,0.8))
#                 move_cmd = {
#                     "x": float(vx),
#                     "y": float(vy), 
#                     "z": float(vyaw)  # 注意：官方使用"z"表示yaw速度
#                 }
#             else:
#                 vx=max(-0.5,min(vx,0.5))  #限制速度不太大不太小
#                 if vx > 0:
#                     vx = max(0.2, vx)  # 正向时不低于 0.2
#                 elif vx < 0:
#                     vx = min(-0.2, vx)  # 负向时不高于 -0.2
#                 vy=max(-0.1,min(vy,0.1))
#                 vyaw=max(-0.8,min(vyaw,0.8))
#                 move_cmd = {
#                     "x": float(vx),
#                     "y": float(vy), 
#                     "z": float(vyaw)  # 注意：官方使用"z"表示yaw速度
#                 }
#             # 创建 Request 消息
#             req = Request()
#             # 设置 API ID
#             req.header.identity.api_id = 1008  # ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW
#             req.parameter = json.dumps(move_cmd, separators=(',', ':'))
#             # 发布 Request 消息
#             self.control_publisher.publish(req)
#         else:
#             self.get_logger().info(f"机器狗已经到达终点")
        



# def main():
#     rclpy.init()
#     vision_node = VisionObstacleDetectionNode('vision_obstacle_detection_node')  # 明确传入节点名称参数
#     rclpy.spin(vision_node)
#     vision_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()