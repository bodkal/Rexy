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
#include <chrono>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(): Node("minimal_publisher")
  {
    this->publisher = this->create_publisher<rexy_msg::msg::LegList>("goal_state", 10);
    this->read_config();
    this->rexy_status = this->home();

    std::cout << rexy_status.legs.size()<<std::endl;
    std::cout << "Ready to start ..." <<std::endl;
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  std::map<std::string, std::vector<int16_t>> name_id_maping;
  rexy_msg::msg::LegList rexy_status;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rexy_msg::msg::LegList>::SharedPtr publisher;

  size_t count_;
  Kinematics rexy_kin;
  int i = 200;

  void read_config()
  {

    YAML::Node config = YAML::LoadFile("/home/koby/rexy_ws/src/rexy_movement_manager/config/rexy_motor_config.yaml");

    std::cout << "read configuriton files ... " << std::endl;
    name_id_maping = config["name_id_maping"].as<std::map<std::string, std::vector<int16_t>>>();
  }

  rexy_msg::msg::LegList home()
  {

    float home_state[3] = {90.0, 30.0, 60.0};
    rexy_msg::msg::LegList tmp;
    rexy_msg::msg::Leg x;

    for (auto const &val : this->name_id_maping)
    {

      x.name = val.first;
      x.pos = {home_state[0], home_state[1], home_state[2]};
      x.vel = {1.0, 1.0, 1.0};
      tmp.legs.push_back(x);
    }
    std::cout << "lode start config ..." <<std::endl;
    std::cout << tmp.legs.size()<<std::endl;

    return tmp;
  }

  void timer_callback()
  {
    if (i > 160)
    {
      std::cout << i << std::endl;
      std::cout <<  this->rexy_status.legs.size() << std::endl;

      this->rexy_status.legs[0] = this->rexy_kin.leg_ik(0, 60, i, this->rexy_status.legs[0].name);

      for (auto const &val : this->rexy_status.legs[0].pos)
      {
        std::cout << val << "\t";
      }
      std::cout << std::endl;
    }
        this->publisher->publish(this->rexy_status);

    i -= 5;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
