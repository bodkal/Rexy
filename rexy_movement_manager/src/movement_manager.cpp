#include <cstdio>
#include <typeinfo>
#include <chrono>
#include <memory>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "rexy_msg/msg/leg.hpp"
#include "rexy_msg/msg/leg_list.hpp"
#include <map>
#include <vector>
#include "yaml-cpp/yaml.h"
#include <math.h>   
#include "kinematics.hpp"   





int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  
  rclcpp::init(argc, argv);
  Kinematics a;
   for(int i = 195; i > 150 ; i-=5){
   auto x= a.leg_ik(0,60,i,"br");
   for(auto const& val: x.pos){
    std::cout<<val<<"\t";
    }
    std::cout<<std::endl;
   }

  
  
 // rclcpp::spin(std::make_shared<MotorControl>());

 
  rclcpp::shutdown();
  
  return 0;
}
