import rclpy
from rclpy.node import Node

from rexy_msg.msg import Sonar
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import matplotlib.pyplot as plt
import numpy as np



class Visualizer(Node):

    def __init__(self):
        super().__init__('visualizer')

        self.publisher_angular_velocity = self.create_publisher(PointStamped, 'angular_velocity_visualizer', 10)
        self.publisher_linear_acc = self.create_publisher(PointStamped, 'linear_acceleration_visualizer', 10)
        self.publisher3_ = self.create_publisher(PointStamped, 'sonar_right', 10)
        self.publisher4_ = self.create_publisher(PointStamped, 'sonar_left', 10)

        self.subscription = self.create_subscription(Imu,'imu_state', self.imu_state_callback, 10)
        self.subscription = self.create_subscription(Sonar,'sonar_state', self.sonar_state_callback, 10)

        self.subscription  # prevent unused variable warning

         # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster_right = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster_left = StaticTransformBroadcaster(self)

        self.create_sonar_transforms(-1,self.tf_static_broadcaster_right)
        self.create_sonar_transforms(1,self.tf_static_broadcaster_left)

    def sonar_state_callback(self, msg):
        pos_stamp = PointStamped()
        pos_stamp.header.frame_id="sonar_right"
        pos_stamp.header.stamp=self.get_clock().now().to_msg()

        msg.right=msg.right if msg.right>0.1 else 3000.0
        pos_stamp.point.x=msg.right/1000
        pos_stamp.point.y=0.0
        pos_stamp.point.z=0.0

        self.publisher3_.publish(pos_stamp)

        pos_stamp = PointStamped()
        pos_stamp.header.frame_id="sonar_left"
        pos_stamp.header.stamp=self.get_clock().now().to_msg()
        msg.left=msg.left if msg.left>0.1 else 3000.0

        pos_stamp.point.x=msg.left/1000
        pos_stamp.point.y=0.0
        pos_stamp.point.z=0.0

        self.publisher4_.publish(pos_stamp)


    def imu_state_callback(self, msg):

        orientation_tf = TransformStamped()

        orientation_tf.header.frame_id="map"
        orientation_tf.header.stamp=self.get_clock().now().to_msg()
        orientation_tf.child_frame_id="rexy_orientation"

        orientation_tf.transform.rotation.x=msg.orientation.x
        orientation_tf.transform.rotation.y=msg.orientation.y
        orientation_tf.transform.rotation.z=msg.orientation.z
        orientation_tf.transform.rotation.w=msg.orientation.w

        self.tf_broadcaster.sendTransform(orientation_tf)

        pos_stamp = PointStamped()
        pos_stamp.header.frame_id="map"
        pos_stamp.header.stamp=self.get_clock().now().to_msg()

        pos_stamp.point.x=msg.linear_acceleration.x
        pos_stamp.point.y=msg.linear_acceleration.y
        pos_stamp.point.z=msg.linear_acceleration.z

        self.publisher_linear_acc.publish(pos_stamp)

        pos_stamp = PointStamped()
        pos_stamp.header.frame_id="map"
        pos_stamp.header.stamp=self.get_clock().now().to_msg()

        pos_stamp.point.x=msg.angular_velocity.x
        pos_stamp.point.y=msg.angular_velocity.y
        pos_stamp.point.z=msg.angular_velocity.z

        self.publisher_angular_velocity.publish(pos_stamp)


    def create_sonar_transforms(self, side,broadcaster):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "rexy_orientation"
        t.child_frame_id =  "sonar_right" if side>0 else "sonar_left"

        t.transform.translation.x = 0.15
        t.transform.translation.y = 0.03*side
        t.transform.translation.z = 0.0
        

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = -0.3826834*side
        t.transform.rotation.w = 0.9238795

        broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    visualizer = Visualizer()

    rclpy.spin(visualizer)

    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
()
