#include "rclcpp/rclcpp.hpp"
#include "rexy_msg/msg/leg.hpp"
#include "rexy_msg/msg/leg_list.hpp"
#include "std_msgs/msg/string.hpp"

#include <motor_control/kinematics.h>

#include "yaml-cpp/yaml.h"
#include <motor_control/LegControl.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <string>
#include <iostream>
#include <regex>

using namespace std::chrono_literals;
using std::placeholders::_1;

class MovementManager
{
public:
  MovementManager()
  {

    this->read_config();
    this->open_motor_connection();

    this->go_home();
    // this->move(this->start_goal, 3,"cartesian");
    // this->print_legs_status();

    this->node = rclcpp::Node::make_shared("movement_manager");
    this->publisher = this->node->create_publisher<rexy_msg::msg::LegList>("goal_state", 10);
    this->joystick_sub = this->node->create_subscription<std_msgs::msg::String>("joystick", 10, std::bind(&MovementManager::joystick_callback, this, _1));
    std::cout << "Ready to start ..." << std::endl;

    this->joystick(this->start_goal);
  }
  void print_goal(const std::map<std::string, tf2::Vector3> new_goal)
  {
    std::cout << "====================================" << std::endl;
    std::cout << "\t\tprint_goal" << std::endl;
    std::cout << "====================================" << std::endl;

    for (const auto &leg : new_goal)
    {
      std::cout << "name: " << leg.first << "[ " << leg.second.x() << "," << leg.second.y() << "," << leg.second.z() << "]" << std::endl;
    }
    std::cout << "====================================" << std::endl;
  }
  void move(const std::map<std::string, tf2::Vector3> &new_goal, int speed, std::string method)
  {

    if (method == "cartesian")
    {
      this->cartesian_move(new_goal, speed);
    }
    else if (method == "forward" || method == "backward")
    {
      this->strate_walk(new_goal, speed, (method == "forward") ? 1 : -1);
    }
    else if (method == "turn_left" || method == "turn_right")
    {
      this->turn(new_goal, speed, (method == "turn_left") ? 1 : -1);
    }
  }

  void turn(const std::map<std::string, tf2::Vector3> &new_goal, float speed, int dir)
  {

    float x_offset = this->turn_x_walk * dir;
    float z_offset = this->turn_z_walk;
    float y_offset = this->turn_y_walk;

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({-x_offset, 0, -z_offset})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({-x_offset, 0, 0})},
                          {"br", new_goal.at("br") + tf2::Vector3({x_offset, -y_offset, 0})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({x_offset, -y_offset, -z_offset})}},
                         speed);

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({-x_offset, 0, 0})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({-x_offset, 0, 0})},
                          {"br", new_goal.at("br") + tf2::Vector3({x_offset, -y_offset, 0})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({x_offset, -y_offset, 0})}},
                         speed);

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({x_offset, 0, 0})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({x_offset, 0, -z_offset})},
                          {"br", new_goal.at("br") + tf2::Vector3({-x_offset, -y_offset, -z_offset})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({-x_offset, -y_offset, 0})}},
                         speed);

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({x_offset, 0, 0})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({x_offset, 0, 0})},
                          {"br", new_goal.at("br") + tf2::Vector3({-x_offset, -y_offset, 0})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({-x_offset, -y_offset, 0})}},
                         speed);
  }

  void strate_walk(const std::map<std::string, tf2::Vector3> &new_goal, float speed, int dir)
  {
    float x_offset = this->forward_x_walk * dir;
    float z_offset = this->forward_z_walk;
    float y_offset = this->forward_y_walk;

    std::vector<int> order({0,1,2,3});
    std::vector<tf2::Vector3> fas_start({{-x_offset-20, 0, -z_offset},
                                         {-x_offset ,   0, 0},
                                         {-x_offset ,   0, 0},
                                         {-x_offset-20, 0, -z_offset}});

    std::vector<tf2::Vector3> fas_mid({{x_offset,  0, -z_offset},
                                       {-x_offset, 0, 0},
                                       {-x_offset, 0, 0},
                                       {x_offset,  0, -z_offset}});

    std::vector<tf2::Vector3> fas_end({{x_offset,    0, 0},
                                       {-x_offset ,  0, 0},
                                       {-x_offset ,  0, 0},
                                       {x_offset,    0, 0}});


    std::cout << "stap: " << (this->w_counter) % 4 << std::endl;
    this->cartesian_move({{"fr", new_goal.at("fr") + fas_start[(order[0] + this->w_counter) % 4]},
                          {"fl", new_goal.at("fl") + fas_start[(order[1] + this->w_counter) % 4]},
                          {"br", new_goal.at("br") + fas_start[(order[2] + this->w_counter) % 4]},
                          {"bl", new_goal.at("bl") + fas_start[(order[3] + this->w_counter) % 4]}},
                         speed);


    this->cartesian_move({{"fr", new_goal.at("fr") + fas_mid[(order[0] + this->w_counter) % 4]},
                          {"fl", new_goal.at("fl") + fas_mid[(order[1] + this->w_counter) % 4]},
                          {"br", new_goal.at("br") + fas_mid[(order[2] + this->w_counter) % 4]},
                          {"bl", new_goal.at("bl") + fas_mid[(order[3] + this->w_counter) % 4]}},
                         speed);

    this->cartesian_move({{"fr", new_goal.at("fr") + fas_end[(order[0] + this->w_counter) % 4]},
                          {"fl", new_goal.at("fl") + fas_end[(order[1] + this->w_counter) % 4]},
                          {"br", new_goal.at("br") + fas_end[(order[2] + this->w_counter) % 4]},
                          {"bl", new_goal.at("bl") + fas_end[(order[3] + this->w_counter) % 4]}},
                         speed);

    this->w_counter += 2;
  }

  void cartesian_move(const std::map<std::string, tf2::Vector3> &new_goal, int speed)
  {
    std::map<std::string, tf2::Vector3> delta;
    std::map<std::string, tf2::Vector3> start;

    tf2::Vector3 error;
    std::map<std::string, int> max_error;
    int legs_max_error = 0;

    // for (const std::string &name : this->legs_name)
    // {
    //   start[name] = this->legs[name].get_pos();
    //   error = (new_goal.at(name) - start.at(name)).absolute();
    //   max_error[name] = int(error[error.maxAxis()]);

    //   delta[name] = ((new_goal.at(name) - start.at(name)) / max_error.at(name));

    //   if (legs_max_error < max_error.at(name))
    //   {
    //     legs_max_error = max_error.at(name);
    //   }
    //   this->legs.at(name).set_new_state(start.at(name));
    // }
    for (const std::string &name : this->legs_name)
    {
      start[name] = this->legs[name].get_pos();
      delta[name] = ((new_goal.at(name) - start.at(name)) / speed);
    }

    rclcpp::Rate rate(10);

    for (int i = 1; i < speed + 1; ++i /* += speed*/)
    {

      for (const std::string &name : this->legs_name)
      {
        this->legs.at(name).set_new_state(start.at(name) + delta[name] * i);

        // if ((max_error.at(name) > 0) & (i <= max_error.at(name)))
        // {
        //   this->legs.at(name).set_new_state(start.at(name) + i * delta.at(name));
        // }
      }
      //  this->print_legs_status();
      rate.sleep();
    }
  }

  std::map<std::string, tf2::Vector3> add(const std::map<std::string, tf2::Vector3> &goal_a,
                                          const std::map<std::string, tf2::Vector3> &goal_b)
  {

    return {{"fr", {goal_a.at("fr").x() + goal_b.at("fr").x(), goal_a.at("fr").y() + goal_b.at("fr").y(), goal_a.at("fr").z() + goal_b.at("fr").z()}},
            {"fl", {goal_a.at("fl").x() + goal_b.at("fl").x(), goal_a.at("fl").y() + goal_b.at("fl").y(), goal_a.at("fl").z() + goal_b.at("fl").z()}},
            {"br", {goal_a.at("br").x() + goal_b.at("br").x(), goal_a.at("br").y() + goal_b.at("br").y(), goal_a.at("br").z() + goal_b.at("br").z()}},
            {"bl", {goal_a.at("bl").x() + goal_b.at("bl").x(), goal_a.at("bl").y() + goal_b.at("bl").y(), goal_a.at("bl").z() + goal_b.at("bl").z()}}};
  }

  std::map<std::string, tf2::Vector3> add(const std::map<std::string, tf2::Vector3> &goal_a,
                                          const tf2::Vector3 &goal_b)
  {

    return {{"fr", {goal_a.at("fr").x() + goal_b.x(), goal_a.at("fr").y() + goal_b.y(), goal_a.at("fr").z() + goal_b.z()}},
            {"fl", {goal_a.at("fl").x() + goal_b.x(), goal_a.at("fl").y() + goal_b.y(), goal_a.at("fl").z() + goal_b.z()}},
            {"br", {goal_a.at("br").x() + goal_b.x(), goal_a.at("br").y() + goal_b.y(), goal_a.at("br").z() + goal_b.z()}},
            {"bl", {goal_a.at("bl").x() + goal_b.x(), goal_a.at("bl").y() + goal_b.y(), goal_a.at("bl").z() + goal_b.z()}}};
  }

  std::map<std::string, tf2::Vector3> add(const std::map<std::string, tf2::Vector3> &goal_a, const tf2::Vector3 &goal_b, const std::string &name)
  {
    std::map<std::string, tf2::Vector3> tmp(goal_a);
    tmp.at(name) = tf2::Vector3({tmp.at(name).x() + goal_b.x(), tmp.at(name).y() + goal_b.y(), tmp.at(name).z() + goal_b.z()});
    return tmp;
  }

  void joystick(std::map<std::string, tf2::Vector3> start_goal)
  {
    rclcpp::Rate rate(50);
    std::map<std::string, std::string> map_input_to_move({{"w", "forward"},
                                                          {"x", "backward"},
                                                          {"d", "turn_left"},
                                                          {"a", "turn_right"},
                                                          {"s", "cartesian"}});
    std::string new_s = "s";
    std::string old_s = "s";
    this->user_shard_string = "s";

    std::regex num("^[0-9]{1,20}$");
    std::regex opr("^[+-]?$");

    int speed = this->start_speed;
    std::cout << "start moving\n\n\tW\nA\tS\tD\n\tX\n\nchange speed - 1 - 20\n+/-z - body up/downn\n+/-f - body forwrd/backword\nO - exit" << std::endl;

    do
    {

      rclcpp::spin_some(this->node);
      new_s = this->user_shard_string;

      if (std::regex_match(new_s, num))
      {
        std::cout << "set new speed ... " << new_s << std::endl;
        speed = std::stoi(new_s);
        this->user_shard_string = old_s;
        new_s = old_s;
      }
      else if ((new_s == "+z") || (new_s == "-z"))
      {
        start_goal = this->add(start_goal, {0, 0, static_cast<float>(44 - int(new_s.front()))});
        this->print_goal(start_goal);
        this->move(start_goal, 1, "cartesian");
      }
      else if ((new_s == "+f") || (new_s == "-f"))
      {
        this->print_goal(start_goal);

        start_goal = this->add(start_goal, {static_cast<float>(44 - int(new_s.front())), 0, 0});
        this->move(start_goal, 1, "cartesian");
      }
      else if (map_input_to_move.count(new_s))
      {
        this->move(start_goal, speed, map_input_to_move.at(new_s));
      }

      old_s = new_s;
      rate.sleep();

    } while (new_s != "o" && rclcpp::ok());
  }

  void wait_for_enter()
  {
    std::cout << "Waiting for enter... " << std::endl;
    std::cin.get();
  }

private:
  std::shared_ptr<rclcpp::Node> node;
  PCA9685 pca9685;
  rexy_msg::msg::LegList rexy_status;
  std::map<std::string, LegControl> legs;
  std::array<std::string, 4> legs_name;
  std::map<std::string, tf2::Vector3> start_goal;

  std::map<std::string, tf2::Transform> legs_transform;
  rclcpp::Publisher<rexy_msg::msg::LegList>::SharedPtr publisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr joystick_sub;

  std::string user_shard_string;
  int start_speed;

  float forward_z_walk;
  float forward_x_walk;
  float forward_y_walk;
  float turn_z_walk;
  float turn_x_walk;
  float turn_y_walk;
  int w_counter = 0;

  void joystick_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    this->user_shard_string = msg->data;
    if (this->user_shard_string != "w")
    {
      this->w_counter = 0;
    }
  }

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

    for (auto &gripper : config["start_pos"].as<std::map<std::string, std::vector<float>>>())
    {
      this->start_goal[gripper.first] = tf2::Vector3({gripper.second[0], gripper.second[1], gripper.second[2]}); // pos
    }

    this->start_speed = config["start_speed"].as<int>();

    this->forward_z_walk = config["forward_z_walk"].as<float>();
    this->forward_x_walk = config["forward_x_walk"].as<float>();
    this->forward_y_walk = config["forward_y_walk"].as<float>();

    this->turn_z_walk = config["turn_z_walk"].as<float>();
    this->turn_x_walk = config["turn_x_walk"].as<float>();
    this->turn_y_walk = config["turn_y_walk"].as<float>();
  }

  void open_motor_connection()
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

    for (const std::string &name : this->legs_name)
    {
      this->legs[name] = LegControl(this->pca9685, name);
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
  MovementManager movement_manager;

  return 0;
}