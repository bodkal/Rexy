import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped

class Visualizer(Node):

    def __init__(self):
        super().__init__('visualizer')

        self.publisher_ = self.create_publisher(PoseStamped, 'orientation_visualizer', 10)
        self.publisher1_ = self.create_publisher(PointStamped, 'angular_velocity_visualizer', 10)
        self.publisher2_ = self.create_publisher(PointStamped, 'linear_acceleration_visualizer', 10)

        self.subscription = self.create_subscription(Imu,'imu_state', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        pos_stamp = PoseStamped()
        pos_stamp.header.frame_id="map"
        pos_stamp.header.stamp=self.get_clock().now().to_msg()

        pos_stamp.pose.orientation.x=msg.orientation.x
        pos_stamp.pose.orientation.y=msg.orientation.y
        pos_stamp.pose.orientation.z=msg.orientation.z
        pos_stamp.pose.orientation.w=msg.orientation.w
        self.publisher_.publish(pos_stamp)

        pos_stamp = PointStamped()
        pos_stamp.header.frame_id="map"
        pos_stamp.header.stamp=self.get_clock().now().to_msg()

        pos_stamp.point.x=msg.linear_acceleration.x
        pos_stamp.point.y=msg.linear_acceleration.y
        pos_stamp.point.z=msg.linear_acceleration.z

        self.publisher1_.publish(pos_stamp)

        pos_stamp = PointStamped()
        pos_stamp.header.frame_id="map"
        pos_stamp.header.stamp=self.get_clock().now().to_msg()

        pos_stamp.point.x=msg.angular_velocity.x
        pos_stamp.point.y=msg.angular_velocity.y
        pos_stamp.point.z=msg.angular_velocity.z

        self.publisher2_.publish(pos_stamp)


def main(args=None):
    rclpy.init(args=args)

    visualizer = Visualizer()

    rclpy.spin(visualizer)

    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
()