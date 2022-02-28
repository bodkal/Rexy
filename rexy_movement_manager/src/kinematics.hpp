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


using namespace std::chrono_literals;

class Kinematics : public rclcpp::Node
{
private:
  int l1;
  int l2;
  int l3;
  
  void read_config();
  float rad_2_deg(float rad);
  float deg_2_rad(float deg);


public:
  rexy_msg::msg::Leg leg_ik(float x,float z,float y,std::string leg_name);
  rexy_msg::msg::Leg leg_ik_v2(float x,float y,float z,std::string leg_name);

  Kinematics();
  
};
