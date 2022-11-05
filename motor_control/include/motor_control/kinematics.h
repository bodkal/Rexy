#ifndef MOTOR_CONTROL_KINEMATICS_H
#define MOTOR_CONTROL_KINEMATICS_H

#include "rexy_msg/msg/leg.hpp"
#include <tf2/LinearMath/Vector3.h>



class Kinematics{
private:
  int l1, l2, l3;

    float rad_2_deg(float rad){
        return rad*180/3.14;
    }

    float deg_2_rad(float deg){
        return deg*3.14/180;
    }


public:
    std::vector<float> leg_inverse_kinematics(const tf2::Vector3 &pos){
        //TODO cainge to pos.dis/dis2
        float yeter = sqrt(pow(pos.z(),2)+pow(pos.y(),2)+pow(pos.x(),2));

        float H = sqrt(pow(yeter,2)-pow(this->l1,2));

        float beta=atan2(pos.z(),pos.y());
        float alfa = atan2(sqrt(pow(pos.z(),2)+pow(pos.y(),2)-pow(this->l1,2)),this->l1);
        float theta1 = alfa - beta;

        float D3 = (pow(H,2)- pow(this->l2,2)-pow(this->l3,2))/(-2*this->l2*this->l3);
        float theta3 = acos(D3);

        //-atan2(sqrt(1-pow(D,2)),D);
        float D2 = (pow(this->l3,2)- pow(this->l2,2)-pow(H,2))/(-2*this->l2*H);

        float theta2 = atan2(pos.x(), sqrt(pow(pos.z(),2)+pow(pos.y(),2))) + acos(D2);//atan2(l3*sin(theta3), l2 + l3*cos(theta3));

        theta1=90-this->rad_2_deg(theta1);
        theta2=90-this->rad_2_deg(theta2);
        theta3=this->rad_2_deg(theta3)-35;

        return {theta1,theta2,theta3};
    }
    Kinematics():l1(60),l2(105),l3(135) {  }
  ~Kinematics()= default;
  
};
#endif //MOTOR_CONTROL_KINEMATICS1_H




