#ifndef MULTI_LIDAR_SLAM
#define MULTI_LIDAR_SLAM

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

class MultiLidarSlam : public rclcpp::Node { 
struct GridParams {
    int width;
        int height;
        double origin_x;
        double origin_y;
        double resolution;
    };

    public:
        MultiLidarSlam();
        void setupParams();
        void initConnections();
        void lidarCallback(const sensor_msgs::msg::LaserScan &lidarMsg);
        void odomCallback(const nav_msgs::msg::Odometry &odomMsg);
    private:
        bool worldToMap(double wx, double wy, int &mx, int &my);
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _mapPublisher;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _lidarSub_Robot01;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSub_robot01;  
        GridParams _gridParams;
        bool hasPose;
        geometry_msgs::msg::PoseStamped latest_pose;
        nav_msgs::msg::OccupancyGrid occupancy_grid;
};


#endif