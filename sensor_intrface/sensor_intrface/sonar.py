import RPi.GPIO as GPIO
import time
from geometry_msgs.msg import TransformStamped 
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import PointStamped

class Sonar:
    def __init__(self,name,ros2_node,GPIO_TRIGGER,GPIO_ECHO,side,max_range=3.0) -> None:
        self._GPIO_TRIGGER=GPIO_TRIGGER
        self._GPIO_ECHO=GPIO_ECHO
        self._max_range=max_range
        self._name=name
        self._ros2_node=ros2_node
        self.tf_static_broadcaster = StaticTransformBroadcaster(ros2_node)
        self.create_sonar_transforms(side)

        #GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)
        #set GPIO direction (IN / OUT)
        GPIO.setup(self._GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self._GPIO_ECHO, GPIO.IN)
        
       

    # Deleting (Calling destructor)
    def __del__(self):
        GPIO.cleanup([self._GPIO_TRIGGER, self._GPIO_ECHO])
        print("close" +self.get_name()+ "cleanly.")
        
    def get_name(self):
        return self._name
    
    def get_dist(self) -> float:
    # set Trigger to HIGH
        GPIO.output(self._GPIO_TRIGGER, GPIO.HIGH)
    
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self._GPIO_TRIGGER, GPIO.LOW)
    
        StartTime = time.time()
        StopTime = time.time()
    
        # save StartTime
        while GPIO.input(self._GPIO_ECHO) == GPIO.LOW:
            StartTime = time.time()
    
        # save time of arrival
        while GPIO.input(self._GPIO_ECHO) == GPIO.HIGH:
            StopTime = time.time()
    
        # time difference between start and arrival
        # multiply with the sonic speed (343 m/s)
        # and divide by 2, because there and back 
        dis =((StopTime - StartTime) * 343) / 2    
        return self.map_to_valid_menstruant(dis)

    def get_dist_as_point(self)->PointStamped:
        dist=self.get_dist()
        return self.create_point_stamp(dist)
            
    def map_to_valid_menstruant(self,new_value) -> float:
        new_value =self._max_range if new_value>self._max_range else new_value
        return new_value

    def create_point_stamp(self,value):
        pos_stamp = PointStamped()
        pos_stamp.header.frame_id=self.get_name()
        pos_stamp.header.stamp=self._ros2_node.get_clock().now().to_msg()

        pos_stamp.point.x=value
        pos_stamp.point.y=0.0
        pos_stamp.point.z=0.0
        return pos_stamp
    
    def create_sonar_transforms(self,side):
        tfs = TransformStamped()

        tfs.header.stamp = self._ros2_node.get_clock().now().to_msg()
        tfs.header.frame_id = "map"
        tfs.child_frame_id = self.get_name()
        
        tfs.transform.translation.x = 0.15
        tfs.transform.translation.y = 0.03*side
        tfs.transform.translation.z = 0.0
        

        tfs.transform.rotation.x = 0.0
        tfs.transform.rotation.y = 0.0
        tfs.transform.rotation.z = -0.3826834*side
        tfs.transform.rotation.w = 0.9238795

        self.tf_static_broadcaster.sendTransform(tfs)
    

