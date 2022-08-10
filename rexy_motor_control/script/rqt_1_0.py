import rclpy
from rclpy.node import Node

from std_msgs.msg import String
#from std_msgs.msg import Int16MultiArray
from rexy_msg.msg import LegList ,Leg
from rexy_msg.srv import LegListState




# bl       deg   pwm   deg   pwm
#motor 11 -> 0 = 130 , 180 = 640 | min 120 max 605
#motor 10 -> 0 = 600 , 180 = 100 | min 100 max 120
#motor 9  -> 25= 550 , 160 = 170 | min 170 max 600

# fl       deg   pwm   deg   pwm
#motor 8 -> 0  = 140 , 180 = 630 | min 130 max 645
#motor 7 -> 0  = 610 , 180 = 125 | min 610 max 110
#motor 6 -> 25 = 90  , 160 = 475 | min 255 max 640

# fr       deg   pwm   deg   pwm
#motor 5 -> 0  = 620 , 180 = 110 | min 620 max 90
#motor 4 -> 0  = 140 , 180 = 635 | min 140 max 600
#motor 3 -> 25 = 620 , 160 = 250 | min 255 max 64038

# br      deg   pwm   deg   pwm
#motor 2 -> 0  = 610 , 180 = 110 | min 605 max 140
#motor 1 -> 0  = 120 , 180 = 610 | min 120 max 590 ziro 200
#motor 0 -> 0  = 180 , 160 = 540 | min 210 max 580 ziro 350


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('micro_rqt')
        self.publisher_ = self.create_publisher(LegList, 'goal_state', 10)
        self.read_state_serves = self.create_client(LegListState, '/get_current_pos')

        while not self.read_state_serves.wait_for_service(timeout_sec=2.0):
            print('service not available, waiting again...')

        all_leg=[]
        names=["br","fr","fl","bl"]
        motors_id=range(12)
        motors_pos=[90.0, 35.0, 60.0, 90.0, 30.0, 60.0, 90.0, 30.0, 60.0, 90.0, 35.0, 60.0]


        for i in range(4):
            number_of_legs=all_leg.__len__()
            all_leg.append(Leg())
            all_leg[-1].name=names[i]
            all_leg[-1].id=motors_id[number_of_legs*3:3+number_of_legs*3]
            all_leg[-1].pos=motors_pos[number_of_legs*3:3+number_of_legs*3]

        self.msg = LegList()
        self.msg.legs=all_leg
        self.timer = self.create_timer(0.5, self.timer_callback)

    def send_request(self):
        future = self.read_state_serves.call_async(LegListState.Request())
        rclpy.spin_until_future_complete(self, future)
        print(future.result())


    def timer_callback(self):
        name=["bl","br","fl","fr"]
        z=int(input(f"input leg : {name}"))

        x=0
        while x>-1:
            y=int(input("input motor : "))
            x=float(input("input angle : "))
            #if x>-1:
            #for i in range(4):
            self.msg.legs[z].pos[y]=x
        #print(x)

        #y=float(input("input id : "))
        #print(f"{y}\t{x}")
        #for i in [1,4,8,7,10]:
        #    self.msg.data[i]+=x
        #for i in [2,5,8,11]:
        #    self.msg.data[i]+=2*x

        #for i in range(4):

        #self.msg.legs[0].vel[0]=1

            print(self.msg)

            self.publisher_.publish(self.msg)

            self.send_request()
        #self.get_logger().info('Publishing: "%s"' % self.msg.data)
       # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
