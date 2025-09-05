#ifndef ODOM_FRAME_PUB_HPP
#define ODOM_FRAME_PUB_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OdomFramePub : public rclcpp::Node {
    public:
        OdomFramePub();
        void setupParams();
        void initConnections();
        void odomCallback(const nav_msgs::msg::Odometry &msg);
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSubscriber;
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr _tfPublisher;


};

#endif