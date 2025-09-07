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
     _robotsNames = this->get_parameter("robot_namespaces").as_string_array();
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
    _listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


void ApriltagTFRemapper::timerCallback() {
    for (const auto &robot: _robotsNames) {
        geometry_msgs::msg::TransformStamped pose;
        try {
          pose = tf_buffer_->lookupTransform(
            "map", robot,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          continue;
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
       

        //original pose
        tf2::Vector3 t_orig(pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z);

        tf2::Quaternion q_orig(
            pose.transform.rotation.x,
            pose.transform.rotation.y,
            pose.transform.rotation.z,
            pose.transform.rotation.w);

        tf2::Transform original_T(q_orig, t_orig);

        // Offset translation (0.08, 0, 0) Offset rotation (yaw +90°)
        tf2::Quaternion q_offset;
        q_offset.setRPY(0, 0, M_PI/2.0);
        tf2::Transform offset_T(
            q_offset, 
            tf2::Vector3(0.08, 0.0, 0.0));

        // --- Compose like Python: new_mat = original_T * offset_trans * offset_rot ---
        tf2::Transform new_mat = original_T * offset_T;

        // --- Extract new translation and rotation ---
        tf2::Vector3 t_new = new_mat.getOrigin();
        tf2::Quaternion q_new = new_mat.getRotation();

        // --- Pack back into TransformStamped ---
        geometry_msgs::msg::TransformStamped odom_to_base_link;
        odom_to_base_link.header.stamp = now();
        odom_to_base_link.header.frame_id = "odom";
        odom_to_base_link.child_frame_id = "base_link";
        odom_to_base_link.transform.translation.x = t_new.x();
        odom_to_base_link.transform.translation.y = t_new.y();
        odom_to_base_link.transform.translation.z = t_new.z();
        odom_to_base_link.transform.rotation.x = q_new.x();
        odom_to_base_link.transform.rotation.y = q_new.y();
        odom_to_base_link.transform.rotation.z = q_new.z();
        odom_to_base_link.transform.rotation.w = q_new.w();

        tf2_msgs::msg::TFMessage tf_msg;
        tf_msg.transforms.push_back(map_to_odom);
        tf_msg.transforms.push_back(odom_to_base_link);
        _robots[robot]._robot_tf_pub->publish(tf_msg);
        //odom publisher
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = t_new.x();
        odom.pose.pose.position.y = t_new.y();
        odom.pose.pose.position.z = t_new.z();
        odom.pose.pose.orientation.x = q_new.x();
        odom.pose.pose.orientation.y = q_new.y();
        odom.pose.pose.orientation.z = q_new.z();
        odom.pose.pose.orientation.w = q_new.w();
        _robots[robot]._odom_pub->publish(odom);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApriltagTFRemapper>());
    rclcpp::shutdown();
    return 0;
}
