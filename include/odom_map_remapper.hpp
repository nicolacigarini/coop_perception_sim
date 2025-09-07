#ifndef ODOM_MAP_REMAPPER_HPP
#define ODOM_MAP_REMAPPER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OdomMapRemapper : public rclcpp::Node {
    struct Params{
        std::string map_frame;
        std::string odom_frame;
        std::string robot_frame;
    };
    public:
        OdomMapRemapper();
        void setupParams();
        void initConnections();
        void odomCallback(const nav_msgs::msg::Odometry &msg);
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSubscriber;
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr _tfPublisher;
        Params _params;

};

#endif