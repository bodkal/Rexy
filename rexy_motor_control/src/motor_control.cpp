#include <typeinfo>

#include <chrono>
#include <memory>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "rexy_msg/msg/leg.hpp"
#include "rexy_msg/msg/leg_list.hpp"
#include <map>
#include <JHPWMDriver/src/JHPWMPCA9685.h>
#include <vector>

using std::placeholders::_1;

using namespace std::chrono_literals;


/*
 loc | motor  |   min     |   max
     |        |deg  pwm   | deg   pwm
-------------------------------------
     |  11    | 0  = 130  | 180 = 640 
 br  |  10    | 0  = 600  | 180 = 100 
     |  9     | 25 = 550  | 160 = 170 
-------------------------------------
     |  8     | 0  = 140  | 180 = 630
 fr  |  7     | 0  = 610  | 180 = 125
     |  6     | 25 = 90   | 160 = 475
-------------------------------------
     |  5     | 0  = 620  | 180 = 110
 fl  |  4     | 0  = 140  | 180 = 635
     |  3     | 25 = 620  | 160 = 250
-------------------------------------
     |  2     | 0  = 610  | 180 = 110
 bl  |  1     | 0  = 120  | 180 = 610
     |  0     | 25 = 180  | 160 = 540
-------------------------------------
*/



class MotorControl : public rclcpp::Node
{
private:

  rclcpp::Publisher<rexy_msg::msg::LegList>::SharedPtr publisher;
  rclcpp::Subscription<rexy_msg::msg::LegList>::SharedPtr subscription;
  rexy_msg::msg::LegList goal_state;
  rexy_msg::msg::LegList current_state;
  rclcpp::TimerBase::SharedPtr timer_;
  PCA9685 *pca9685 = new PCA9685();

/*
  int16_t min_pwm_val[12]={180,120,610,620,140,620,90,610,140,550,600,130};
  int16_t max_pwm_val[12]={540,610,110,250,635,110,475,125,630,170,100,640};

  int16_t min_angle_val[12]={25,0,0,25,0,0,25,0,0,25,0,0};
  int16_t max_angle_val[12]={160,180,180,160,180,180,160,180,180,160,180,180};


  int16_t min_pwm_val[12]={180,120,610,620,140,620,90,610,140,550,600,640};
  int16_t max_pwm_val[12]={540,610,110,250,635,110,475,125,630,170,100,130};

  int16_t min_angle_val[12]={25,0,0,25,0,0,25,0,0,25,0,0};
  int16_t max_angle_val[12]={160,180,180,160,180,180,160,180,180,160,180,180};
 */ 
std::map<std::string, std::vector<int16_t> > name_id_maping = {{ "bl", {0,1,2}  },
                                                               { "fl", {3,4,5}  },
                                                               { "fr", {6,7,8}  },
                                                               { "br", {9,10,11}}};

std::map<std::string, std::vector<int16_t> > min_angle =      {{ "bl", {25,0,0}  },
                                                               { "fl", {25,0,0}  },
                                                               { "fr", {25,0,0}  },
                                                               { "br", {25,0,0}}};

std::map<std::string, std::vector<int16_t> > max_angle =      {{ "bl", {160,180,180} },
                                                               { "fl", {160,180,180} },
                                                               { "fr", {160,180,180} },
                                                               { "br", {160,180,180}}};

std::map<std::string, std::vector<int16_t> > min_pwm =        {{ "bl", {180,120,610} },
                                                               { "fl", {620,140,620} },
                                                               { "fr", {90,610,140}  },
                                                               { "br", {550,600,130}}};

std::map<std::string, std::vector<int16_t> > max_pwm =        {{ "bl", {540,610,110} },
                                                               { "fl", {250,635,110} },
                                                               { "fr", {475,125,630} },
                                                               { "br", {170,100,640}}};




  float next_step(float goal, float current,float step){

      if (abs(goal-current)>=1){
      if (std::signbit(goal-current)){
        return current - step;
      }
      return current + step;

    }
  return current;
  }


  void publish_state()
  {

    for(int leg = 0;leg < 4;leg++){

    std::vector<short> current_id =  this->name_id_maping[this->current_state.legs[leg].name];

    for (int motor = 0;motor < 3;motor++){
      this->current_state.legs[leg].vel[motor]=this->goal_state.legs[leg].vel[motor];
      this->current_state.legs[leg].pos[motor]=
      this-> next_step(this->goal_state.legs[leg].pos[motor],this->current_state.legs[leg].pos[motor],this->current_state.legs[leg].vel[motor]);

      int pwm_send_to_motor=convert_agnle_to_pwm(this->current_state.legs[leg].pos[motor],motor,this->current_state.legs[leg].name);
      this->pca9685->setPWM(current_id[motor], 0, pwm_send_to_motor);

      //std::cout << this->current_state.legs[leg].pos[motor]<<'\t';
      std::cout << this->current_state.legs[leg].name<<'\t';

    }
    }
    std::cout << std::endl;

    this->publisher->publish(this->current_state);

  }


  rexy_msg::msg::LegList home(){

    float home_state[3]={90.0, 30.0, 60.0};
    //float home_state[12]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    rexy_msg::msg::LegList tmp;
    rexy_msg::msg::Leg x;

    for(auto const& val: name_id_maping)
    {
      x.name=val.first;
      x.pos={home_state[0],home_state[1],home_state[2]};
      x.vel={1.0,1.0,1.0};
      tmp.legs.push_back(x);
    }
    return tmp;

  }
  void goal_state_callback(const rexy_msg::msg::LegList::SharedPtr msg)
  {
      for (int i =0; i<4;i++){
      this->goal_state.legs[i]=msg->legs[i];
    }

  }

  int interpolation(double x, int in_min, int in_max, int out_min, int out_max)
{
  return round((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

  int convert_agnle_to_pwm(double angele,int id,std::string name)
  {
    auto miv_vec=this->min_pwm[name];
    auto max_vec=this->max_pwm[name];

    auto miv_ang=this->min_angle[name];
    auto max_ang=this->max_angle[name];

    int min_pwm = std::min(miv_vec[id],max_vec[id]);
    int max_pwm = std::max(miv_vec[id],max_vec[id]);


    
    int goal_pwm=this->interpolation(angele,miv_ang[id],max_ang[id],miv_vec[id],max_vec[id]);

    if (goal_pwm<min_pwm){
      return min_pwm;
    }
    if (goal_pwm>max_pwm){
    return max_pwm;
    }
    return goal_pwm;
  }

  void stop_motor()
  {
    pca9685->closePCA9685();
  }

public:
 
  MotorControl() : Node("motor_control")
  {
    this->goal_state=this->home();
    this->current_state=this->home();

    this->publisher = this->create_publisher<rexy_msg::msg::LegList>("cuurent_state", 10);
    this->subscription = this->create_subscription<rexy_msg::msg::LegList>("goal_state", 10, std::bind(&MotorControl::goal_state_callback, this, _1));


    int err = pca9685->openPCA9685();
    printf("%d\n", err);

    if (err < 0)
    {
      printf("Error: %d", pca9685->error);
      pca9685->closePCA9685();
    }
    else
    {
      pca9685->setAllPWM(0, 0);
      pca9685->reset();
      pca9685->setPWMFrequency(60);
    }

    this->timer_ = this->create_wall_timer(40ms, std::bind(&MotorControl::publish_state, this));
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControl>());
  printf("exit");
  rclcpp::shutdown();
  return 0;
}
