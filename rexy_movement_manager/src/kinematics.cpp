#include "kinematics.hpp"

using namespace std::chrono_literals;


void Kinematics::read_config(){
    
    this->l1 = 60;
    this->l2 = 105;
    this->l3 = 135;
    
    /*
    YAML::Node config = YAML::LoadFile("/home/koby/rexy_ws/src/rexy_motor_control/config/rexy_motor_config.yaml");
                   std::cout <<"read configuriton files ... "<<std::endl;

                // loop over the positions Rectangle and print them:
              std::map<std::string, std::vector<int16_t> > name_id_maping = config["name_id_maping"].as<std::map<std::string, std::vector<int16_t> >>();
              
              std::vector<int16_t> min_pwm_val = config["map_angel_pwm"]["min_pwm_val"].as<std::vector<int16_t>>();
              std::vector<int16_t> max_pwm_val = config["map_angel_pwm"]["max_pwm_val"].as<std::vector<int16_t>>();
              std::vector<int16_t> min_angle_val = config["map_angel_pwm"]["min_angle_val"].as<std::vector<int16_t>>();
              std::vector<int16_t> max_angle_val = config["map_angel_pwm"]["max_angle_val"].as<std::vector<int16_t>>();
              
               std::cout << name_id_maping["br"][2]<<std::endl;
               std::cout << min_pwm_val[0]<<std::endl;
               std::cout << max_pwm_val[0]<<std::endl;
               std::cout << min_angle_val[0]<<std::endl;
               std::cout << max_angle_val[0]<<std::endl;*/
  }

float Kinematics::rad_2_deg(float rad){
return rad*180/3.14;
}

float Kinematics::deg_2_rad(float deg){
return deg*3.14/180;
}


rexy_msg::msg::Leg Kinematics::leg_ik(float x,float y,float z,std::string leg_name){ 
    float beta=atan2(z,y);
    float H = sqrt(pow(z,2)+pow(y,2)-pow(l1,2));
    float alfa = atan2(H,l1);
    float theta1 =alfa-beta;


    float D = (pow(H,2)+pow(x,2) - pow(l2,2)-pow(l3,2))/(2*l2*l3);
    float theta3 = -atan2(sqrt(1-pow(D,2)),D);

    float theta2 = atan2(x, H) - atan2(l3*sin(theta3), l2 + l3*cos(theta3));
  
    /*
    std::cout<<"t1 : "<<90-this->rad_2_deg(theta1)<<std::endl;
    std::cout<<"t2 : "<<this->rad_2_deg(theta2)<<std::endl;
    std::cout<<"t3 : "<<140+this->rad_2_deg(theta3)<<std::endl;
    */

    rexy_msg::msg::Leg leg;
    leg.name=leg_name;
    leg.pos={ 90-this->rad_2_deg(theta1),
             this->rad_2_deg(theta2),
             140+this->rad_2_deg(theta3)};
             
    leg.vel={1.0,1.0,1.0};
    return leg;
  }
  
  
  Kinematics::Kinematics() : Node("motor_control")
  {
    this->read_config();
  }


