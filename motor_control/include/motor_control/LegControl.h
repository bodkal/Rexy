//
// Created by picko on 10/16/22.
//

#ifndef SRC_LEGCONTROL_H
#define SRC_LEGCONTROL_H

#include <iostream>
#include <unistd.h>

#include "rexy_msg/msg/leg.hpp"
#include "yaml-cpp/yaml.h"
#include "motor_control/kinematics.h"
#include <tf2/LinearMath/Vector3.h>
#include <JHPWMDriver/src/JHPWMPCA9685.h>


class LegControl:  public Kinematics{
public:
    LegControl() = default;
    LegControl(PCA9685 &motors_ref,const std::string &leg_name);

    ~LegControl() = default;
    void print_state();
    rexy_msg::msg::Leg get_state();
    tf2::Vector3 get_pos();
    void set_new_state(const tf2::Vector3 &new_state);

    void go_home();
    void send_pwm_to_motor_no_defence(int motor,int pwm);
    std::string name;

private:

    tf2::Vector3 home_pos;

    tf2::Vector3 current_pos;
    std::array<int,3> min_pwm_val,max_pwm_val;
    std::array<float,3> min_angle_val, max_angle_val;

    rexy_msg::msg::Leg home_state;
    rexy_msg::msg::Leg leg_state;

     std::shared_ptr<PCA9685> motor_drivers;
    std::array<int,3> motors;
    int convert_angle_to_pwm(double angele,int id);

    int interpolation(double x, int in_min, int in_max, int out_min, int out_max);

    void send_angel_to_motor(const rexy_msg::msg::Leg &new_state_angel);
    void read_config();





};



#endif //SRC_LEGCONTROL_H
