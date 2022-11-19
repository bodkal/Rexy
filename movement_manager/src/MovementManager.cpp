#include "rclcpp/rclcpp.hpp"
#include "rexy_msg/msg/leg.hpp"
#include "rexy_msg/msg/leg_list.hpp"

#include <motor_control/kinematics.h>

#include "yaml-cpp/yaml.h"
#include <motor_control/LegControl.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher() : Node("minimal_publisher")
  {
    this->publisher = this->create_publisher<rexy_msg::msg::LegList>("goal_state", 10);
    this->read_config();
    this->open_motor_connectin();
    //  std::map<std::string, tf2::Vector3> new_goal;

    for (const std::string &name : this->legs_name)
    {
      this->legs[name] = LegControl(this->pca9685, name);
    }

    this->go_home();
    this->print_legs_status();

    std::cout << "Ready to start ..." << std::endl;
  }

  void move(std::map<std::string, tf2::Vector3> &new_goal, std::string method)
  {
    if (method == "cartesian")
    {
      this->cartesian_move(new_goal, 1);
    }
  }
  
  void step_move(){
    for (const std::string &name : this->legs_name)
    {
      this->legs.at(name).set_new_state(start.at(name));
    }
  }
  void cartesian_move(std::map<std::string, tf2::Vector3> &new_goal, int speed)
  {
    std::map<std::string, tf2::Vector3> delta;
    std::map<std::string, tf2::Vector3> start;

    tf2::Vector3 error;
    std::map<std::string, int> max_error;
    int legs_max_error = 0;

    for (const std::string &name : this->legs_name)
    {
      start[name] = this->legs[name].get_pos();
      error = (new_goal.at(name) - start.at(name)).absolute();
      max_error[name] = int(error[error.maxAxis()]);

      delta[name] = ((new_goal.at(name) - start.at(name)) / max_error.at(name));

      if (legs_max_error < max_error.at(name))
      {
        legs_max_error = max_error.at(name);
      }
      this->legs.at(name).set_new_state(start.at(name));
    }
    std::cout << "legs_max_error: " << legs_max_error << std::endl;


      rclcpp::Rate rate(100);
      for (int i = 0; i < legs_max_error; i += speed)
      {
        std::cout << "i: " << i << std::endl;

        for (const std::string &name : this->legs_name)
        {
          std::cout << "name: " << name << " max_error.at(name):" << max_error.at(name) << std::endl;
          if (max_error.at(name)>0 & i <= max_error.at(name))
          {
            this->legs.at(name).set_new_state(start.at(name) + i * delta.at(name));
          }
        }
        this->print_legs_status();
        rate.sleep();
      }
    }

private:
  std::array<int, 2> test;
  PCA9685 pca9685; 
  rexy_msg::msg::LegList rexy_status;
  std::map<std::string, LegControl> legs;
  std::array<std::string, 4> legs_name;
  std::map<std::string, tf2::Vector3> leg_goals;

  std::map<std::string, tf2::Transform> legs_transform;
  rclcpp::Publisher<rexy_msg::msg::LegList>::SharedPtr publisher;

  void print_legs_status()
  {
    std::cout << "---------------------" << std::endl;
    for (const std::string &name : this->legs_name)
    {
      legs[name].print_state();
    }
    std::cout << "---------------------" << std::endl;
  }
  void read_config()
  {
    std::string file_path = std::string(get_current_dir_name()) + "/src/movement_manager/config/movement_manager_config.yaml";
    std::cout << file_path << std::endl;

    YAML::Node config = YAML::LoadFile(file_path);

    this->legs_name = config["legs_name"].as<std::array<std::string, 4>>();

    for (auto &gripper : config["legs_transform"].as<std::map<std::string, std::vector<float>>>())
    {
      this->legs_transform[gripper.first] = tf2::Transform(
          {gripper.second[3], gripper.second[4], gripper.second[5], gripper.second[6]}, // ori
          {gripper.second[0], gripper.second[1], gripper.second[2]});                   // pos
    }

    this->test = config["test"].as<std::array<int, 2>>();
  }

  void open_motor_connectin()
  {
    int err = this->pca9685.openPCA9685();
    // printf("%d\n", err);

    if (err < 0)
    {
      printf("Error: %d", this->pca9685.error);
      this->pca9685.closePCA9685();
    }
    else
    {
      this->pca9685.setAllPWM(0, 0);
      this->pca9685.reset();
      this->pca9685.setPWMFrequency(60);
    }
  }

  void go_home()
  {
    for (std::string const &name : legs_name)
    {
      this->legs[name].go_home();
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  MinimalPublisher a;
  Kinematics kin;
  std::cout << "enter to start" << std::endl;
  std::map<std::string, tf2::Vector3> new_goal({{"fr", {0, 60, 160}},
                                                {"fl", {0, 60, 160}},
                                                {"br", {0, 60, 160}},
                                                {"bl", {0, 60, 160}}});

  std::map<std::string, tf2::Vector3> new_goal1({{"fr", {0, 60, 160}},
                                                 {"fl", {0, 60, 160}},
                                                 {"br", {0, 60, 160}},
                                                 {"bl", {0, 60, 160}}});

  std::map<std::string, tf2::Vector3> new_goal2({{"fr", {0, 60, 120}},
                                                 {"fl", {0, 60, 160}},
                                                 {"br", {0, 60, 160}},
                                                 {"bl", {0, 60, 160}}});

  a.move(new_goal, "cartesian");

  std::cout << "enter to start" << std::endl;
  std::cin.get();

  a.move(new_goal1, "cartesian");
  a.move(new_goal2, "cartesian");
  a.move(new_goal1, "cartesian"); /*
    a.move(new_goal2,"cartesian");
    a.move(new_goal1,"cartesian");
    a.move(new_goal2,"cartesian");
    a.move(new_goal1,"cartesian");
    a.move(new_goal2,"cartesian");
    a.move(new_goal1,"cartesian");
  */
  return 0;
}
