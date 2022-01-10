#include <typeinfo>

#include <chrono>
#include <memory>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "rexy_msg/msg/leg.hpp"
#include "rexy_msg/msg/leg_list.hpp"
#include <map>
#include <JHPWMDriver/src/JHPWMPCA9685.h>
#include <vector>

using std::placeholders::_1;

using namespace std::chrono_literals;


/*
 loc | motor  |   min     |   max
     |        |deg  pwm   | deg   pwm
-------------------------------------
     |  11    | 0  = 130  | 180 = 640 
 bl  |  10    | 0  = 600  | 180 = 100 
     |  9     | 25 = 550  | 160 = 170 
-------------------------------------
     |  8     | 0  = 140  | 180 = 630
 fl  |  7     | 0  = 610  | 180 = 125
     |  6     | 25 = 90   | 160 = 475
-------------------------------------
     |  5     | 0  = 620  | 180 = 110
 fr  |  4     | 0  = 140  | 180 = 635
     |  3     | 25 = 620  | 160 = 250
-------------------------------------
     |  2     | 0  = 610  | 180 = 110
 br  |  1     | 0  = 120  | 180 = 610
     |  0     | 25 = 180  | 160 = 540
-------------------------------------
*/


class MotorControl : public rclcpp::Node
{
private:

  rclcpp::Publisher<rexy_msg::msg::LegList>::SharedPtr publisher;
  rclcpp::Subscription<rexy_msg::msg::LegList>::SharedPtr subscription;
  rexy_msg::msg::LegList goal_state;
  rexy_msg::msg::LegList current_state;
  rclcpp::TimerBase::SharedPtr timer_;
  PCA9685 *pca9685 = new PCA9685();


  int16_t min_pwm_val[12]={180,120,610,620,140,620,90,610,140,550,600,130};
  int16_t max_pwm_val[12]={540,610,110,250,635,110,475,125,630,170,100,640};

  int16_t min_angle_val[12]={25,0,0,25,0,0,25,0,0,25,0,0};
  int16_t max_angle_val[12]={160,180,180,160,180,180,160,180,180,160,180,180};


std::map<std::string, std::vector<int16_t> > name_id_maping = {{ "bl", {0,1,2}  },
                                                               { "fl", {3,4,5}  },
                                                               { "fr", {6,7,8}  },
                                                               { "br", {9,10,11}}};



  void publish_state()
  {
    int x=1; 

    while (x>0){
  std::cout << "Type a number: "<<std::endl; // Type a number and press enter
  std::cin >> x; // Get user input from the keyboard
  std::cout << " x "<<std::endl; // Type a number and press enter

  this->pca9685->setPWM(0, 0,x);
    }
  }

 
public:
 
  MotorControl() : Node("motor_control")
  {
   
    int err = pca9685->openPCA9685();
    printf("%d\n", err);

    if (err < 0)
    {
      printf("Error: %d", pca9685->error);
      pca9685->closePCA9685();
    }
    else
    {
      pca9685->setAllPWM(0, 0);
      pca9685->reset();
      pca9685->setPWMFrequency(60);
    }
    this->publish_state();
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControl>());
  printf("exit");
  rclcpp::shutdown();
  return 0;
}
