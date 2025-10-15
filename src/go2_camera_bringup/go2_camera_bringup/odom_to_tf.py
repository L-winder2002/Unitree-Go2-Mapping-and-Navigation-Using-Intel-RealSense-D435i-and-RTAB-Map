#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/robot_odom_fixed')
        self.declare_parameter('fallback_parent', 'odom')
        self.declare_parameter('fallback_child',  'base_link')

        topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.sub = self.create_subscription(Odometry, topic, self.cb, 50)
        self.br  = TransformBroadcaster(self)
        self.get_logger().info(f'[odom_to_tf] subscribing: {topic}')

    def cb(self, msg: Odometry):
        parent = msg.header.frame_id or self.get_parameter('fallback_parent').get_parameter_value().string_value
        child  = msg.child_frame_id or self.get_parameter('fallback_child').get_parameter_value().string_value

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = parent
        tf.child_frame_id  = child
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(tf)

def main():
    rclpy.init()
    rclpy.spin(OdomToTF())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
