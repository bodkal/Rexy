#include "rclcpp/rclcpp.hpp"
#include "rexy_msg/msg/leg.hpp"
#include "rexy_msg/msg/leg_list.hpp"

#include <motor_control/kinematics.h>

#include "yaml-cpp/yaml.h"
#include <motor_control/LegControl.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <ncurses.h>

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

  void move(const std::map<std::string, tf2::Vector3> &new_goal, std::string method)
  {
    if (method == "cartesian")
    {
      this->cartesian_move(new_goal, 2);
    }
    if (method == "forward")
    {
      this->strate_walk(new_goal, 2, 1);
    }
    if (method == "backward")
    {
      this->strate_walk(new_goal, 2, -1);
    }
  }

  // void step_move(){
  //   for (const std::string &name : this->legs_name)
  //   {
  //     this->legs.at(name).set_new_state(this->start.at(name));
  //   }
  // }

  void strate_walk(const std::map<std::string, tf2::Vector3> &new_goal, float speed, int dir)
  {
    float x_offset = 70 * dir;
    float z_offset = 25;

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({x_offset / 2, 0, -z_offset})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({-x_offset / 2, 0, 0})},
                          {"br", new_goal.at("br") + tf2::Vector3({-x_offset / 2, 0, 0})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({x_offset / 2, 0, -z_offset})}},
                         speed);

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({x_offset / 2, 0, 0})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({-x_offset / 2, 0, 0})},
                          {"br", new_goal.at("br") + tf2::Vector3({-x_offset / 2, 0, 0})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({x_offset / 2, 0, 0})}},
                         speed);

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({-x_offset / 2, 0, 0})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({x_offset / 2, 0, -z_offset})},
                          {"br", new_goal.at("br") + tf2::Vector3({x_offset / 2, 0, -z_offset})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({-x_offset / 2, 0, 0})}},
                         speed);

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({-x_offset / 2, 0, 0})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({x_offset / 2, 0, 0})},
                          {"br", new_goal.at("br") + tf2::Vector3({x_offset / 2, 0, 0})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({-x_offset / 2, 0, 0})}},
                         speed);
  }

  void cartesian_move(const std::map<std::string, tf2::Vector3> &new_goal, int speed)
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

    rclcpp::Rate rate(110 - speed * 10);

    for (int i = 0; i < legs_max_error; i += speed)
    {
      // std::cout << "i: " << i << std::endl;

      for (const std::string &name : this->legs_name)
      {
        if ((max_error.at(name) > 0) & (i <= max_error.at(name)))
        {
          this->legs.at(name).set_new_state(start.at(name) + i * delta.at(name));
        }
      }
      //  this->print_legs_status();
      rate.sleep();
    }
  }

  void joystick(std::map<std::string, tf2::Vector3> start_goal)
  {
    rclcpp::Rate rate(100);

    std::string s;
    std::cout << "\tW\nA\t\tD\n\tX\n\nO" << std::endl;
    do
    {
      std::cin >> s;
      if (s == "w")
      {
        this->move(start_goal, "forward");
      }
      else if (s == "x")
      {
        this->move(start_goal, "backward");
      }
      rate.sleep();
    } while (s != "o");
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
  float x = -10;
  float y = 60;
  std::map<std::string, tf2::Vector3> start_goal({{"fr", {x, y, 150}},
                                                  {"fl", {x, y, 150}},
                                                  {"br", {x, y + 10, 140}},
                                                  {"bl", {x, y + 10, 140}}});
  a.move(start_goal, "cartesian");

  std::cout << "enter to start" << std::endl;

  std::cin.get();
  a.joystick(start_goal);

  return 0;
}
