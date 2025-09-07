#ifndef MULTI_LIDAR_SLAM
#define MULTI_LIDAR_SLAM

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include <unordered_map>

class MultiLidarSlam : public rclcpp::Node { 
    struct GridParams {
    int width;
        int height;
        double origin_x;
        double origin_y;
        double resolution;
    };
    struct RobotState {
        bool hasPose{false};
        geometry_msgs::msg::PoseStamped latest_pose;
    };


    public:
        MultiLidarSlam();
        void setupParams();
        void initConnections();
        void addNewRobot(const std::string &robotName);
        void elaborateScan(const sensor_msgs::msg::LaserScan &lidarMsg, const geometry_msgs::msg::PoseStamped &poseMsg);

    private:
        bool worldToMap(double wx, double wy, int &mx, int &my);
        void raytrace(int x0, int y0, int x1, int y1);
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _mapPublisher;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _lidarSub_Robot01;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSub_robot01;  


        std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> _scan_subs;
        std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> _odom_subs;
        GridParams _gridParams;
        nav_msgs::msg::OccupancyGrid occupancy_grid;
        std::unordered_map<std::string, RobotState> _robots;
};


#endif