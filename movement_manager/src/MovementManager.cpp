#include "rclcpp/rclcpp.hpp"
#include "rexy_msg/msg/leg.hpp"
#include "rexy_msg/msg/leg_list.hpp"
#include "yaml-cpp/yaml.h"
#include <motor_control/LegControl.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(): Node("minimal_publisher")
  {
    this->publisher = this->create_publisher<rexy_msg::msg::LegList>("goal_state", 10);
    this->read_config();
    this->open_motor_connectin();
    //  std::map<std::string, tf2::Vector3> new_goal;


      for (const std::string &name : this->legs_name){
          this->legs[name]=LegControl(this->pca9685,name);
          this->leg_goals[name]=tf2::Vector3(40,60,150);

         // new_goal[name]=tf2::Vector3(40,60,150);
      }

      this->print_legs_status();
      this->legs.at("br").send_pwm_to_motor_no_defence(this->test[0],this->test[1]);

      std::cout<<"name: "<<this->legs.at("fl").name<<std::endl;
  //    this->go_home();
 //     this->set_goal(this->leg_goals,"cartesian");
//    this->print_legs_status();


    std::cout << "Ready to start ..." <<std::endl;
  }

    void set_goal( std::map<std::string, tf2::Vector3> &new_goal,std::string method){
//      this->leg_goals=new_goal;
      if (method == "cartesian"){
          cartesian_move(new_goal);
      }
  }

  void cartesian_move( std::map<std::string, tf2::Vector3> &new_goal) {
      std::map <std::string, tf2::Vector3> delta;
      std::map <std::string, tf2::Vector3> start;

      tf2::Vector3 error;
      std::map<std::string, int> max_error;
      int legs_max_error = 0;

      for (const std::string &name: this->legs_name) {
          start[name] = this->legs[name].get_pos();
          error = (new_goal.at(name) - start.at(name)).absolute();
          max_error[name] = int(error[error.maxAxis()]);
          delta[name] = ((new_goal.at(name) - start.at(name)) / max_error.at(name));

          if (legs_max_error < max_error.at(name)) {
              legs_max_error = max_error.at(name);
          }
      }
      for (int i = 0; i < legs_max_error; ++i) {
          for (const std::string &name: this->legs_name) {
              if (i <= max_error.at(name)) {

                  this->legs.at(name).set_new_state(start.at(name) + i * delta.at(name));
              }
          }
          this->print_legs_status();
      }
  }

private:
std::array<int,2> test;
  PCA9685 pca9685;// = new PCA9685();
    rexy_msg::msg::LegList rexy_status;
  std::map<std::string,LegControl> legs;
  std::array<std::string,4> legs_name;
  std::map<std::string, tf2::Vector3> leg_goals;

  std::map<std::string, tf2::Transform> legs_transform;
  rclcpp::Publisher<rexy_msg::msg::LegList>::SharedPtr publisher;

    void print_legs_status(){
        std::cout<<"---------------------"<<std::endl;
        for (const std::string &name : this->legs_name){
            legs[name].print_state();
        }
        std::cout<<"---------------------"<<std::endl;

    }
  void read_config()
  {
    std::string file_path = std::string(get_current_dir_name())+"/src/movement_manager/config/movement_manager_config.yaml";
      std::cout << file_path << std::endl;

      YAML::Node config = YAML::LoadFile(file_path);

      this->legs_name = config["legs_name"].as<std::array<std::string,4>>();

      for ( auto &gripper: config["legs_transform"].as<std::map<std::string ,std::vector<float>>>()) {
          this->legs_transform[gripper.first]=tf2::Transform(
                  {gripper.second[3],gripper.second[4],gripper.second[5],gripper.second[6]},   // ori
                  {gripper.second[0],gripper.second[1], gripper.second[2]});                      // pos
      }

      this->test = config["test"].as<std::array<int,2>>();

  }

void open_motor_connectin(){
    int err = this->pca9685.openPCA9685();
    //printf("%d\n", err);

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
      for (std::string const &name : legs_name){
          this->legs[name].go_home();
      }
  }

};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  MinimalPublisher a;


  /*
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();*/
  return 0;
}
