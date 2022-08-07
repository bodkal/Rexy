//
// Created by koby on 8/7/22.
//

#ifndef REXY_MOTOR_CONTROL_MOTOR_CONTROL_H
#define REXY_MOTOR_CONTROL_MOTOR_CONTROL_H

#endif //REXY_MOTOR_CONTROL_MOTOR_CONTROL_H


#include <typeinfo>

#include <chrono>
#include <memory>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "rexy_msg/msg/leg.hpp"
#include "rexy_msg/msg/leg_list.hpp"

#include "rexy_msg/srv/LegListState.hpp"


#include <map>
#include <JHPWMDriver/src/JHPWMPCA9685.h>
#include <vector>




class MotorControl : public rclcpp::Node {
private:

    rclcpp::Service<rexy_msg::srv::LegListState>::SharedPtr current_state_service;
    rclcpp::Subscription<rexy_msg::msg::LegList>::SharedPtr subscription;

    rexy_msg::msg::LegList goal_state;
    rexy_msg::msg::LegList current_state;
    rclcpp::TimerBase::SharedPtr timer_;
    PCA9685 *pca9685 = new PCA9685();

    std::vector <int16_t> min_pwm_val;
    std::vector <int16_t> max_pwm_val;
    std::vector <int16_t> min_angle_val;
    std::vector <int16_t> max_angle_val;
    std::vector<float> home_state;

    std::map <std::string, std::vector<int16_t>> name_id_maping;

    void current_pos_service(const std::shared_ptr<rexy_msg::srv::LegListState::Request> request,
                                           std::shared_ptr<rexy_msg::srv::LegListState:Response> response);

    void read_config();

    void publish_state();

    rexy_msg::msg::LegList home();

    void goal_state_callback(const rexy_msg::msg::LegList::SharedPtr msg);

    int interpolation(double x, int in_min, int in_max, int out_min, int out_max);

    int convert_agnle_to_pwm(double angele,int id);

    void stop_motor();

public:

    MotorControl() : Node("motor_control");

};


