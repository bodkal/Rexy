import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_intrface.sonar import Sonar
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

class SonarPublisher(Node):

    def __init__(self):
        super().__init__('sonar_publisher')
        self.sonar_left = Sonar(name="sonar_left",ros2_node=self,GPIO_TRIGGER = 6,GPIO_ECHO = 5,side=-1)
        self.sonar_right= Sonar(name="sonar_right",ros2_node = self,GPIO_TRIGGER = 4,GPIO_ECHO = 17,side=1)

        self.String_msg = String()


        self.publisher1_ = self.create_publisher(PointStamped, '/sonar_right', 10)
        self.publisher2_ = self.create_publisher(PointStamped, '/sonar_left', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        print("start timers",flush=True)

    def timer_callback(self):
        self.publisher1_.publish(self.sonar_right.get_dist_as_point())
        self.publisher2_.publish(self.sonar_left.get_dist_as_point())






def main(args=None):
    print("insilize ... ",flush=True)
    rclpy.init(args=args)

    sonar_publisher =SonarPublisher()
    rclpy.spin(sonar_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

