//
// Created by picko on 10/16/22.
//

#ifndef MOVEMENT_MANAGER_STABILITYCONTROL_H
#define MOVEMENT_MANAGER_STABILITYCONTROL_H

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/transform_broadcaster.h"

#include <iostream>
#include <tf2/LinearMath/Transform.h>

class StabilityControl

{
public:
    StabilityControl(rclcpp::Node::SharedPtr &node_ref) : node(node_ref)
    {
        this->imu_state_sub = this->node->create_subscription<sensor_msgs::msg::Imu>("/imu_state", 10, std::bind(&StabilityControl::imu_state_callback, this, std::placeholders::_1));

        this->tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*(this->node));
    }
    ~StabilityControl() = default;

    void imu_state_callback(const sensor_msgs::msg::Imu::Ptr msg)
    {

        geometry_msgs::msg::TransformStamped orientation_tf;

        orientation_tf.header.frame_id = "map";
        orientation_tf.header.stamp = this->node->get_clock()->now();
        orientation_tf.child_frame_id = "rexy_orientation";

        orientation_tf.transform.rotation.x = msg->orientation.x;
        orientation_tf.transform.rotation.y = msg->orientation.y;
        orientation_tf.transform.rotation.z = msg->orientation.z;
        orientation_tf.transform.rotation.w = msg->orientation.w;

        tf2::Transform rexy_ori({msg->orientation.x,
                                 msg->orientation.y,
                                 msg->orientation.z,
                                 msg->orientation.w},
                                {0, 0, 0});

        tf2::Vector3 rexy_ori_z_axis = rexy_ori * tf2::Vector3(0, 0, 1);

        std::cout << rexy_ori_z_axis << std::endl;

       this->tf_broadcaster->sendTransform(orientation_tf);
    }

private:
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_state_sub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

#endif // MOVEMENT_MANAGER_STABILITYCONTROL_H
