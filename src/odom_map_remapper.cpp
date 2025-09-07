#ifndef ODOM_MAP_REMAPPER_CPP
#define ODOM_MAP_REMAPPER_CPP
#include "../include/odom_map_remapper.hpp"

OdomMapRemapper::OdomMapRemapper() : Node("odom_map_remapper",
                                                rclcpp::NodeOptions()
                                                .allow_undeclared_parameters(true)
                                                .automatically_declare_parameters_from_overrides(true))
{
    setupParams();
    initConnections();
}

void OdomMapRemapper::setupParams() {
    RCLCPP_INFO(this->get_logger(), "here");
    _params.map_frame = this->get_parameter("map_frame").as_string();
    _params.odom_frame = this->get_parameter("odom_frame").as_string();
    _params.robot_frame = this->get_parameter("robot_frame").as_string();
}

void OdomMapRemapper::initConnections() {
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string tf_topic = this->get_parameter("tf_topic").as_string();
    _odomSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&OdomMapRemapper::odomCallback, this, std::placeholders::_1));
    _tfPublisher = this->create_publisher<tf2_msgs::msg::TFMessage>(tf_topic,10);
}

void OdomMapRemapper::odomCallback(const nav_msgs::msg::Odometry &msg) {

    // Transform publisher from robot base_link to odom
    geometry_msgs::msg::TransformStamped odom_base_link;
    odom_base_link.header.stamp = now();
    odom_base_link.header.frame_id = _params.odom_frame;
    odom_base_link.child_frame_id = _params.robot_frame;
    odom_base_link.transform.translation.x = msg.pose.pose.position.x;
    odom_base_link.transform.translation.y = msg.pose.pose.position.y;
    odom_base_link.transform.translation.z = msg.pose.pose.position.z;
    odom_base_link.transform.rotation = msg.pose.pose.orientation;


    /*Fixed tf publisher from map to odom. 
    In our setup, we assume to have perfect localization and to ignore odometry, therefore map = odom at every step
    This definition is however consistent with REP 105
    */
    geometry_msgs::msg::TransformStamped map_odom;
    map_odom.header.stamp = now();
    map_odom.header.frame_id = _params.map_frame;
    map_odom.child_frame_id = _params.odom_frame;
    map_odom.transform.translation.x = 0;
    map_odom.transform.translation.y = 0;
    map_odom.transform.translation.z = 0;
    map_odom.transform.rotation.x = 0;
    map_odom.transform.rotation.y = 0;
    map_odom.transform.rotation.z = 0;
    map_odom.transform.rotation.w = 1;

    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(odom_base_link);
    tf_msg.transforms.push_back(map_odom);
    _tfPublisher->publish(tf_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomMapRemapper>());
    rclcpp::shutdown();
    return 0;
}
#endif