#include "../include/apriltag_to_tf_remapper.hpp"

ApriltagTFRemapper::ApriltagTFRemapper() : Node("apriltag_tf_remapper",
                                                rclcpp::NodeOptions()
                                                .allow_undeclared_parameters(true)
                                                .automatically_declare_parameters_from_overrides(true))
{
    setupParams();
    initConnections();   
}

void ApriltagTFRemapper::setupParams() {
     _robotsNames = this->get_parameter("robots_namespaces").as_string_array();
}

void ApriltagTFRemapper::initConnections() {
    for (const auto &robot : _robotsNames) {
        auto robotHandler = RobotHandler();
        robotHandler.robot_namespace = robot;
        robotHandler._odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/"+robot+"/pose",10);
        robotHandler._robot_tf_pub = this->create_publisher<tf2_msgs::msg::TFMessage>("/"+robot+"tf",10);
        _robots[robot] = robotHandler;
    }   

    _tfTimer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ApriltagTFRemapper::timerCallback, this));
    tf_buffer_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
}


void ApriltagTFRemapper::timerCallback() {
    for (const auto &robot: _robotsNames) {
        geometry_msgs::msg::TransformStamped t;
        try {
          t = tf_buffer_->lookupTransform(
            "map", robot,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          return;
        }
        
        //map to odom fixed transform
        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.stamp = now();
        map_to_odom.header.frame_id = "map";
        map_to_odom.child_frame_id = "odom";
        map_to_odom.transform.translation.x = 0;
        map_to_odom.transform.translation.y = 0;
        map_to_odom.transform.translation.z = 0;
        map_to_odom.transform.rotation.x = 0;
        map_to_odom.transform.rotation.y = 0;
        map_to_odom.transform.rotation.z = 0;
        map_to_odom.transform.rotation.w = 1;
        //odom to base_link transform
        //odom publisher


    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApriltagTFRemapper>());
    rclcpp::shutdown();
    return 0;
}
