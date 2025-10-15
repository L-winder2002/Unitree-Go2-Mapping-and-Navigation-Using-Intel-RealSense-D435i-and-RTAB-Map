import rclpy, copy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

class OdomTimeSync(Node):
    def __init__(self):
        super().__init__('odom_time_sync')

        self.in_odom  = self.declare_parameter('in_odom', '/utlidar/robot_odom').get_parameter_value().string_value
        self.out_odom = self.declare_parameter('out_odom', '/robot_odom_fixed').get_parameter_value().string_value
        self.rgb_topic = self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw').get_parameter_value().string_value

        self.get_logger().info(f"[OdomTimeSync] 同步 {self.in_odom} → {self.out_odom}，相机={self.rgb_topic}")

        self.pub = self.create_publisher(Odometry, self.out_odom, 10)
        self.sub_odom = self.create_subscription(Odometry, self.in_odom, self.cb_odom, 10)
        self.sub_rgb  = self.create_subscription(Image, self.rgb_topic, self.cb_rgb, 10)

        self.last_rgb_stamp = None
        self.printed_once = False  # 自己加一个一次性标志

    def cb_rgb(self, msg: Image):
        self.last_rgb_stamp = msg.header.stamp
        if not self.printed_once:
            self.get_logger().info(f"✅ 收到第一帧相机图像 stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
            self.printed_once = True

    def cb_odom(self, msg: Odometry):
        if self.last_rgb_stamp is None:
            self.get_logger().warn("还未收到相机时间戳，暂不发布同步数据")
            return
        new_msg = copy.deepcopy(msg)
        new_msg.header.stamp = self.last_rgb_stamp
        self.pub.publish(new_msg)

def main():
    rclpy.init()
    node = OdomTimeSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
