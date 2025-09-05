#ifndef MULTI_LIDAR_SLAM_CPP
#define MULTI_LIDAR_SLAM_CPP

#include "../include/multi_lidar_slam.hpp"



MultiLidarSlam::MultiLidarSlam() : Node("multi_lidar_slam",
                                                rclcpp::NodeOptions()
                                                .allow_undeclared_parameters(true)
                                                .automatically_declare_parameters_from_overrides(true))
{
    setupParams();
    initConnections();
}

void MultiLidarSlam::setupParams() {
    _gridParams.width = 200;
    _gridParams.height = 200;
    _gridParams.resolution = 0.05;
    _gridParams.origin_x = -(_gridParams.width*_gridParams.resolution)/2.0;
    _gridParams.origin_y = -(_gridParams.height*_gridParams.resolution)/2.0;
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.resolution = _gridParams.resolution;
    occupancy_grid.info.width = _gridParams.width;
    occupancy_grid.info.height = _gridParams.height;
    occupancy_grid.info.origin.position.x = _gridParams.origin_x;
    occupancy_grid.info.origin.position.y = _gridParams.origin_y;
    occupancy_grid.data.assign(_gridParams.width*_gridParams.height,-1);
}

void MultiLidarSlam::initConnections() {    
    _lidarSub_Robot01 = this->create_subscription<sensor_msgs::msg::LaserScan>("/robot1/lidar/scan", 10, std::bind(&MultiLidarSlam::lidarCallback, this, std::placeholders::_1));
    _odomSub_robot01 = this->create_subscription<nav_msgs::msg::Odometry>("/robot1/sim_ground_truth_pose", 10, std::bind(&MultiLidarSlam::odomCallback, this, std::placeholders::_1));
    _mapPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",10);
}

void MultiLidarSlam::lidarCallback(const sensor_msgs::msg::LaserScan &msg){
    if(!hasPose) {
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000, "No pose received, skipping scan");
        return;
    }

    const double px = latest_pose.pose.position.x;
    const double py = latest_pose.pose.position.y;
    tf2::Quaternion q(
        latest_pose.pose.orientation.x,
        latest_pose.pose.orientation.y,
        latest_pose.pose.orientation.z,
        latest_pose.pose.orientation.w);
    tf2::Matrix3x3 R(q);

    double angle = msg.angle_min + M_PI_2;

    for (size_t i = 0; i < msg.ranges.size(); ++i, angle+=msg.angle_increment) {
        float r = msg.ranges[i];
        
        double lx = r*std::cos(angle);
        double ly = r*std::sin(angle);

        double mx = R[0][0]*lx + R[0][1]*ly + px;
        double my = R[1][0]*lx + R[1][1]*ly + py;

        int gx, gy;
        if(!worldToMap(mx,my,gx,gy))
            continue;
        size_t index = gy*_gridParams.width + gx;
        occupancy_grid.data[index] = 100;
    }
    occupancy_grid.header.stamp = now();
    _mapPublisher->publish(occupancy_grid);

}

bool MultiLidarSlam::worldToMap(double wx, double wy, int &mx, int &my) {
    mx = static_cast<int>((wx-_gridParams.origin_x)/_gridParams.resolution);
    my = static_cast<int>((wy-_gridParams.origin_y)/_gridParams.resolution);
    return (mx>=0 && mx<_gridParams.width && my>=0 && my<_gridParams.height);
}

void MultiLidarSlam::odomCallback(const nav_msgs::msg::Odometry &msg) {
    latest_pose.pose = msg.pose.pose;
    hasPose = true;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiLidarSlam>());
    rclcpp::shutdown();
    return 0;
}

#endif