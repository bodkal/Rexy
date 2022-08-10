#include "rexy_motor_control/motor_control.h"

using std::placeholders::_1;

using namespace std::chrono_literals;

    void MotorControl::current_pos_service(const std::shared_ptr<rexy_msg::srv::LegListState::Request> request,
                                   std::shared_ptr<rexy_msg::srv::LegListState:Response> response)
    {
        for (int i =0; i<4;i++) {
            response.legs[i] = this->current_state->legs[i];
        }
    }


    void MotorControl::read_config(){

    YAML::Node config = YAML::LoadFile("/home/rexy/rexy_ws/src/rexy_motor_control/config/rexy_motor_config.yaml");
                   std::cout <<"read configuriton files ... "<<std::endl;

                // loop over the positions Rectangle and print them:
              name_id_maping = config["name_id_maping"].as<std::map<std::string, std::vector<int16_t> >>();

              min_pwm_val = config["map_angel_pwm"]["min_pwm_val"].as<std::vector<int16_t>>();
              max_pwm_val = config["map_angel_pwm"]["max_pwm_val"].as<std::vector<int16_t>>();
              min_angle_val = config["map_angel_pwm"]["min_angle_val"].as<std::vector<int16_t>>();
              max_angle_val = config["map_angel_pwm"]["max_angle_val"].as<std::vector<int16_t>>();
              home_state = config["home"]["pos"].as<std::vector<float>>();
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


  rexy_msg::msg::LegList motor_control::home(){

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

  int motor_control::convert_agnle_to_pwm(double angele,int id)
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


public:

    MotorControl::MotorControl() : Node("motor_control")
  { 
    this->read_config();

    this->goal_state=this->home();
    this->current_state=this->home();

    this->subscription = this->create_subscription<rexy_msg::msg::LegList>
            ("goal_state", 10, std::bind(&MotorControl::goal_state_callback, this, _1));

    this->current_state_service = this->create_service<rexy_msg::srv::LegListState>
            ("get_current_pos", &current_pos_service);

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
};



int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControl>());
  printf("exit");
  rclcpp::shutdown();
  return 0;
}



#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

    /* class ServiceServerExample //{ */

    class ServiceServerExample : ~/
    {
    public:
        ServiceServerExample(rclcpp::NodeOptions options);

    private:
        // | --------------------- service servers -------------------- |

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_;

        void callback_set_bool(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                     std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    };

    //}

    /* ServiceServerExample() //{ */

    ServiceServerExample::ServiceServerExample(rclcpp::NodeOptions options) : Node("service_server_example", options)
    {

        RCLCPP_INFO(get_logger(), "[ServiceServerExample]: initializing");

        // | --------------------- service server --------------------- |

        service_server_ = create_service<std_srvs::srv::SetBool>(
                "~/set_bool_in", std::bind(&ServiceServerExample::callback_set_bool, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "[ServiceServerExample]: initialized");
    }

    //}

    // | ------------------------ callbacks ----------------------- |

    /* callback_set_bool() //{ */

    void ServiceServerExample::callback_set_bool(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                                 std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        RCLCPP_INFO(get_logger(), "[ServiceServerExample]: received service call: %s", request->data ? "TRUE" : "FALSE");

        response->message = "succeeded";
        response->success = true;
    }

    //}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ServiceServerExample)
