#include "rexy_motor_control/motor_control.h"

using namespace std::placeholders;
using namespace std::chrono_literals;

    void MotorControl::current_pos_service(const std::shared_ptr<rexy_msg::srv::LegListState::Request> request,
                                                std::shared_ptr<rexy_msg::srv::LegListState::Response> response)
    {

        std::cout<<request<<std::endl; //just for the worning

        for (int i =0; i<4;i++) {
            response->legs[i] = this->current_state.legs[i];
        }

    }


    void MotorControl::read_config(){

    YAML::Node config = YAML::LoadFile("/home/rexy/rexy_ws/src/rexy_motor_control/config/rexy_motor_config.yaml");
                   std::cout <<"read configuriton files ... "<<std::endl;

                // loop over the positions Rectangle and print them:
              this->name_id_maping = config["name_id_maping"].as<std::map<std::string, std::vector<int16_t> >>();

              this->min_pwm_val = config["map_angel_pwm"]["min_pwm_val"].as<std::vector<int16_t>>();
              this->max_pwm_val = config["map_angel_pwm"]["max_pwm_val"].as<std::vector<int16_t>>();
              this->min_angle_val = config["map_angel_pwm"]["min_angle_val"].as<std::vector<int16_t>>();
              this->max_angle_val = config["map_angel_pwm"]["max_angle_val"].as<std::vector<int16_t>>();
              this->home_state = config["home"]["pos"].as<std::vector<float>>();
  }


  void MotorControl::publish_state()
  {
    for(int leg = 0;leg < 4;leg++){

    std::vector<short> current_id =  this->name_id_maping[this->current_state.legs[leg].name];

    for (int motor_index = 0;motor_index < 3;motor_index++){

      int pwm_to_motor=this->convert_agnle_to_pwm(this->goal_state.legs[leg].pos[motor_index],current_id[motor_index]);

      this->pca9685->setPWM(current_id[motor_index],
                            0,
                            pwm_to_motor);

      std::cout << this->current_state.legs[leg].name<<" "<<motor_index<<" "<<current_id[motor_index]<<"  "<<
      convert_agnle_to_pwm(this->current_state.legs[leg].pos[motor_index],current_id[motor_index])<<"\t";

    }
    this->current_state.legs[leg].pos=this->goal_state.legs[leg].pos;
    }
    std::cout << std::endl;

  }


  rexy_msg::msg::LegList MotorControl::home(){

    rexy_msg::msg::LegList tmp_legs;
    rexy_msg::msg::Leg tmp_leg;

    for(auto const& val: name_id_maping)
    {
      tmp_leg.name=val.first;
      tmp_leg.pos={this->home_state[0],this->home_state[1],this->home_state[2]};
      tmp_legs.legs.push_back(tmp_leg);
    }
    return tmp_legs;

  }
  void MotorControl::goal_state_callback(const rexy_msg::msg::LegList::SharedPtr msg)
  {
      for (int i =0; i<4;i++){
      this->goal_state.legs[i]=msg->legs[i];
    }

  }

  int MotorControl::interpolation(double x, int in_min, int in_max, int out_min, int out_max)
{
  return round((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

  int MotorControl::convert_agnle_to_pwm(double angele,int id)
  {
    int min_pwm = std::min(this->min_pwm_val[id],this->max_pwm_val[id]);
    int max_pwm = std::max(this->min_pwm_val[id],this->max_pwm_val[id]);
    
    int goal_pwm=this->interpolation(angele,this->min_angle_val[id],this->max_angle_val[id],this->min_pwm_val[id],this->max_pwm_val[id]);

    if (goal_pwm<min_pwm){return min_pwm;}
    if (goal_pwm>max_pwm){return max_pwm;}

    return goal_pwm;
  }

  void MotorControl::stop_motor()
  {
    pca9685->closePCA9685();
  }



    MotorControl::MotorControl() : Node("motor_control")
  { 
    this->read_config();

    this->goal_state=this->home();
    this->current_state=this->home();

    this->subscription = this->create_subscription<rexy_msg::msg::LegList>
            ("goal_state", 10, std::bind(&MotorControl::goal_state_callback, this, _1));

    this->current_state_service = this->create_service<rexy_msg::srv::LegListState>
            ("get_current_pos",std::bind(&MotorControl::current_pos_service,this,_1,_2));

    int err = pca9685->openPCA9685();

    if (err < 0){
        printf("Error: %d", pca9685->error);
        pca9685->closePCA9685();
    }
    else{
        pca9685->setAllPWM(0, 0);
        pca9685->reset();
        pca9685->setPWMFrequency(60);
    }

    this->timer_ = this->create_wall_timer(40ms, std::bind(&MotorControl::publish_state, this));
    
  }



int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControl>());
  printf("exit");
  rclcpp::shutdown();
  return 0;
}


