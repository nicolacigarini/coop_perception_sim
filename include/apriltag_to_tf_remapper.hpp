#ifndef APRILTAG_TO_TF_REMAPPER_HPP
#define APRILTAG_TO_TF_REMAPPER_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "unordered_map"

class ApriltagTFRemapper : public rclcpp::Node {
    struct RobotHandler {
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr _robot_tf_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
        std::string robot_namespace;
    };
    public:
        ApriltagTFRemapper();
        void initConnections();
        void setupParams();
        void timerCallback();



    private:
        std::unordered_map<std::string, RobotHandler> _robots;
        std::vector<std::string> _robotsNames;
        rclcpp::TimerBase::SharedPtr _tfTimer;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> _listener;
};

#endif