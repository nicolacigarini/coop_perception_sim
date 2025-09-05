#ifndef ODOM_FRAME_PUB_CPP
#define ODOM_FRAME_PUB_CPP
#include "../include/odom_frame_pub.hpp"

OdomFramePub::OdomFramePub() : Node("odom_frame_publisher",
                                                rclcpp::NodeOptions()
                                                .allow_undeclared_parameters(true)
                                                .automatically_declare_parameters_from_overrides(true))
{
    setupParams();
    initConnections();
}

void OdomFramePub::setupParams() {

}

void OdomFramePub::initConnections() {
    _odomSubscriber = this->create_subscription<nav_msgs::msg::Odometry>("sim_ground_truth_pose", 10, std::bind(&OdomFramePub::odomCallback, this, std::placeholders::_1));
    _tfPublisher = this->create_publisher<tf2_msgs::msg::TFMessage>("tf",10);
}

void OdomFramePub::odomCallback(const nav_msgs::msg::Odometry &msg) {
    geometry_msgs::msg::TransformStamped odom_base_link;
    odom_base_link.header.stamp = now();
    odom_base_link.header.frame_id = "odom";
    odom_base_link.child_frame_id = "base_link";
    odom_base_link.transform.translation.x = msg.pose.pose.position.x;
    odom_base_link.transform.translation.y = msg.pose.pose.position.y;
    odom_base_link.transform.translation.z = msg.pose.pose.position.z;
    odom_base_link.transform.rotation = msg.pose.pose.orientation;

    geometry_msgs::msg::TransformStamped map_odom;
    map_odom.header.stamp = now();
    map_odom.header.frame_id = "map";
    map_odom.child_frame_id = "odom";
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
    rclcpp::spin(std::make_shared<OdomFramePub>());
    rclcpp::shutdown();
    return 0;
}

#endif