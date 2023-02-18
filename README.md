# rexy 
**open sorch**

https://github.com/mike4192/spotMicro


## micro ros

**Micro ros install:**

https://micro.ros.org/

https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/

https://manzurmurshid.medium.com/how-to-connect-teensy-3-2-with-micro-ros-and-ros2-foxy-6c8f99c9b66a

**Run micro ros agent:**

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
##


**Enable csi camera**

https://chuckmails.medium.com/enable-pi-camera-with-raspberry-pi4-ubuntu-20-10-327208312f6e
##

**opencv install**

https://www.geeksforgeeks.org/how-to-install-opencv-in-c-on-linux/

##
** Use orb_slam**
'''
export LD_LIBRARY_PATH=~/Pangolin/build/src/:~/ORB_SLAM3/Thirdparty/DBoW2/lib:~/ORB_SLAM3/Thirdparty/g2o/lib:~/ORB_SLAM3/lib:$LD_LIBRARY_PATH
ros2 run orbslam3 mono /home/picko/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/picko/rexy_ws/src/orbslam3/src/monocular/TUM1.yaml
'''