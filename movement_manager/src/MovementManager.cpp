#include "rclcpp/rclcpp.hpp"
#include "rexy_msg/msg/leg.hpp"
#include "rexy_msg/msg/leg_list.hpp"

#include <motor_control/kinematics.h>

#include "yaml-cpp/yaml.h"
#include <motor_control/LegControl.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <string>
#include <iostream>
#include <thread>
#include <regex>


using namespace std::chrono_literals;

class MovementManager : public rclcpp::Node
{
public:
  MovementManager() : Node("movement_manager")
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

  void move(const std::map<std::string, tf2::Vector3> &new_goal, int speed, std::string method)
  {

    if (method == "cartesian")
    {
      this->cartesian_move(new_goal, speed);
    }
    if (method == "forward")
    {
      this->strate_walk(new_goal, speed, 1);
    }
    if (method == "backward")
    {
      this->strate_walk(new_goal, speed, -1);
    }
    if (method == "turn_left")
    {
      this->turn(new_goal, speed, 1);
    }
    if (method == "turn_right")
    {
      this->turn(new_goal, speed, -1);
    }
  }

  // void step_move(){
  //   for (const std::string &name : this->legs_name)
  //   {
  //     this->legs.at(name).set_new_state(this->start.at(name));
  //   }
  // }

  void turn(const std::map<std::string, tf2::Vector3> &new_goal, float speed, int dir)
  {

    float x_offset = 25 * dir;
    float z_offset = 25;

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({-x_offset, 0, -z_offset})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({-x_offset, 0, 0})},
                          {"br", new_goal.at("br") + tf2::Vector3({x_offset, 0, 0})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({x_offset, 0, -z_offset})}},
                         speed);

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({-x_offset, 0, 0})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({-x_offset, 0, 0})},
                          {"br", new_goal.at("br") + tf2::Vector3({x_offset, 0, 0})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({x_offset, 0, 0})}},
                         speed);

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({x_offset, 0, 0})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({x_offset, 0, -z_offset})},
                          {"br", new_goal.at("br") + tf2::Vector3({-x_offset, 0, -z_offset})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({-x_offset, 0, 0})}},
                         speed);

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({x_offset, 0, 0})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({x_offset, 0, 0})},
                          {"br", new_goal.at("br") + tf2::Vector3({-x_offset, 0, 0})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({-x_offset, 0, 0})}},
                         speed);
  }

  void strate_walk(const std::map<std::string, tf2::Vector3> &new_goal, float speed, int dir)
  {
    float x_offset = 35 * dir;
    float z_offset = 25;

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({x_offset, 0, -z_offset})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({-x_offset, 0, 0})},
                          {"br", new_goal.at("br") + tf2::Vector3({-x_offset, 0, 0})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({x_offset, 0, -z_offset})}},
                         speed);

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({x_offset, 0, 0})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({-x_offset, 0, 0})},
                          {"br", new_goal.at("br") + tf2::Vector3({-x_offset, 0, 0})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({x_offset, 0, 0})}},
                         speed);

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({-x_offset, 0, 0})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({x_offset, 0, -z_offset})},
                          {"br", new_goal.at("br") + tf2::Vector3({x_offset, 0, -z_offset})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({-x_offset, 0, 0})}},
                         speed);

    this->cartesian_move({{"fr", new_goal.at("fr") + tf2::Vector3({-x_offset, 0, 0})},
                          {"fl", new_goal.at("fl") + tf2::Vector3({x_offset, 0, 0})},
                          {"br", new_goal.at("br") + tf2::Vector3({x_offset, 0, 0})},
                          {"bl", new_goal.at("bl") + tf2::Vector3({-x_offset, 0, 0})}},
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

    rclcpp::Rate rate(100);

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

std::map<std::string, tf2::Vector3> combine_goals(const std::map<std::string, tf2::Vector3>  &goal_a,
                                                  const std::map<std::string, tf2::Vector3>  &goal_b){

return {{"fr", {goal_a.at("fr").x()+goal_b.at("fr").x(), goal_a.at("fr").y()+goal_b.at("fr").y(), goal_a.at("fr").z()+goal_b.at("fr").z()}},
        {"fl", {goal_a.at("fl").x()+goal_b.at("fl").x(), goal_a.at("fl").y()+goal_b.at("fl").y(), goal_a.at("fl").z()+goal_b.at("fl").z()}},
        {"br", {goal_a.at("br").x()+goal_b.at("br").x(), goal_a.at("br").y()+goal_b.at("br").y(), goal_a.at("br").z()+goal_b.at("br").z()}},
        {"bl", {goal_a.at("bl").x()+goal_b.at("bl").x(), goal_a.at("bl").y()+goal_b.at("bl").y(), goal_a.at("bl").z()+goal_b.at("bl").z()}}} ;


                                                  }


  void joystick(std::map<std::string, tf2::Vector3> start_goal)
  {
    rclcpp::Rate rate(100);

    std::string new_s = "s";
    std::string old_s = "s";
    this->user_shard_string = "s";
    std::cout << "set speed: 1 - 20" << std::endl;
    std::cin >> new_s;
    std::regex num("^[0-9]{1,20}$");
    std::regex opr("^[+-]?$");

    int speed = std::stoi(new_s);
    std::cout << "start moving\n\n\tW\nA\tS\tD\n\tX\n\nchange speed - 1 - 20\n+ - body up\n- - body down\nO - exit" << std::endl;

    std::thread user_input_thread(&MovementManager::read_char_from_user, this);


    do
    {
      
     new_s=this->user_shard_string;

      
      if (std::regex_match(new_s, num)){
            std::cout << "set new speed ... "<<new_s << std::endl;
            speed = std::stoi(new_s);
            this->user_shard_string=old_s;
            new_s=old_s;
      }
      else if (new_s == "+"){
      std::map<std::string, tf2::Vector3> up_goal({{"fr", {0, 0, 2}},
                                                   {"fl", {0, 0, 2}},
                                                   {"br", {0, 0, 2}},
                                                   {"bl", {0,0, 2}}});
      start_goal = this->combine_goals(start_goal,up_goal);
      this->move(start_goal, speed, "cartesian");

      }
      else if (new_s == "-"){
      std::map<std::string, tf2::Vector3> down_goal({{"fr", {0, 0, -2}},
                                                     {"fl", {0, 0, -2}},
                                                     {"br", {0, 0, -2}},
                                                     {"bl", {0,0, -2}}});

      start_goal = this->combine_goals(start_goal,down_goal);
      this->move(start_goal, speed, "cartesian");
      }
      else if (new_s == "w"){this->move(start_goal, speed, "forward"); }
      else if (new_s == "x"){this->move(start_goal, speed, "backward"); }
      else if (new_s == "d"){this->move(start_goal, speed, "turn_left");}
      else if (new_s == "a"){this->move(start_goal, speed, "turn_right");}
      else if (new_s == "s"){this->move(start_goal, speed, "cartesian"); }
      
      old_s=new_s;
      rate.sleep();

    } while (new_s != "o" && rclcpp::ok());

    user_input_thread.join();

  }

  void wait_for_enter()
  {
    std::cout << "Waiting for enter... " << std::endl;
    std::cin.get();
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
  std::string user_shard_string;

void read_char_from_user()
{
    rclcpp::Rate rate(20);
    std::cout << "open to user input " << std::endl;

  do{
   std::cin>>this->user_shard_string;
  rate.sleep();

} while (this->user_shard_string != "o" && rclcpp::ok());

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
  MovementManager a;
  Kinematics kin;
  float x = -10;
  float y = 60;
  std::map<std::string, tf2::Vector3> start_goal({{"fr", {x, y, 150}},
                                                  {"fl", {x, y, 150}},
                                                  {"br", {x, y + 10, 140}},
                                                  {"bl", {x, y + 10, 140}}});
  a.move(start_goal,1, "cartesian");

  a.wait_for_enter();
  a.joystick(start_goal);

  return 0;
}
