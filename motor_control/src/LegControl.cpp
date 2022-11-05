#include <motor_control/LegControl.h>


  int LegControl::interpolation(double x, int in_min, int in_max, int out_min, int out_max)
{
  return round((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

    int LegControl::convert_angle_to_pwm(double angele,int id){
    int min_pwm = std::min(this->min_pwm_val[id],this->max_pwm_val[id]);
    int max_pwm = std::max(this->min_pwm_val[id],this->max_pwm_val[id]);

    int goal_pwm=this->interpolation(angele,
                                     this->min_angle_val[id],
                                     this->max_angle_val[id],
                                     this->min_pwm_val[id],
                                     this->max_pwm_val[id]);

    if (goal_pwm<min_pwm){
    return min_pwm;
    }
    else if (goal_pwm>max_pwm){
    return max_pwm;
    }
    return goal_pwm;
  }

//TODO need to create small steps?
  void LegControl::go_home()
  {
    this->current_pos=this->home_pos;
    this->send_angel_to_motor(this->home_state);
  }

void LegControl::print_state(){
std::cout<<"name: "<<this->leg_state.name<<"\t pos: ["<<this->leg_state.pos[0]<<", "<<this->leg_state.pos[1]<<", "<<this->leg_state.pos[2]<<"]"<<std::endl;
}

void LegControl::read_config(){
    std::string file_path = std::string(get_current_dir_name())+"/src/motor_control/config/leg_config.yaml";
    YAML::Node config = YAML::LoadFile(file_path);

    this->home_state.name=this->name;
    this->home_state.pos  = config[this->name]["home"].as<std::vector<float>>();

    this->motors=config[this->name]["motors"].as<std::array<int,3>>();

    std::array<float,3> tmp =config[this->name]["home_pos"].as<std::array<float,3>>();
    this->home_pos  =tf2::Vector3(tmp[0],tmp[1],tmp[2]);

    this->leg_state.name=this->name;

    this->min_pwm_val  = config[this->name]["min_pwm"].as<std::array<int,3>>();
    this->max_pwm_val  = config[this->name]["max_pwm"].as<std::array<int,3>>();
    this->min_angle_val  = config[this->name]["min_angle"].as<std::array<float,3>>();
    this->max_angle_val  = config[this->name]["max_angle"].as<std::array<float,3>>();
}

LegControl::LegControl(PCA9685 &motors_ref,const std::string  &leg_name):
 motor_drivers(&motors_ref),
 name(leg_name)
  {

    this->read_config();
    this->go_home();
}

  void LegControl::send_angel_to_motor(const rexy_msg::msg::Leg &new_state){
      for (int motor_index = 0;motor_index < 3;motor_index++){
          int pwm_to_motor=this->convert_angle_to_pwm(new_state.pos[motor_index],motor_index);
          this->motor_drivers->setPWM(motors[motor_index], 0, pwm_to_motor);
          std::cout<<"angel: "<<new_state.pos[motor_index]<<"\tpwm:"<<pwm_to_motor<<std::endl;
      }
      this->leg_state=new_state;
  }

void LegControl::set_new_state(const tf2::Vector3 &pos_new_state){
      rexy_msg::msg::Leg new_state;
      new_state.name=this->name;
      new_state.pos=this->leg_inverse_kinematics(pos_new_state);
      this->current_pos={pos_new_state};
      this->send_angel_to_motor(new_state);

}

tf2::Vector3 LegControl::get_pos(){
return this->current_pos;
}

rexy_msg::msg::Leg LegControl::get_state(){
    return this->leg_state;
}

    void LegControl::send_pwm_to_motor_no_defence(int motor,int pwm){
                this->motor_drivers->setPWM(this->motors[0]+motor, 0, pwm);
    }

