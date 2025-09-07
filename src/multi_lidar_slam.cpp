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
    _gridParams.width = this->get_parameter("grid.width").as_int();
    _gridParams.height = this->get_parameter("grid.height").as_int();
    _gridParams.resolution = this->get_parameter("grid.resolution").as_double();
    _gridParams.origin_x = -(_gridParams.width*_gridParams.resolution)/2.0;
    _gridParams.origin_y = -(_gridParams.height*_gridParams.resolution)/2.0;


    occupancy_grid.header.frame_id = this->get_parameter("grid.frame").as_string();
    occupancy_grid.info.resolution = _gridParams.resolution;
    occupancy_grid.info.width = _gridParams.width;
    occupancy_grid.info.height = _gridParams.height;
    occupancy_grid.info.origin.position.x = _gridParams.origin_x;
    occupancy_grid.info.origin.position.y = _gridParams.origin_y;
    occupancy_grid.data.assign(_gridParams.width*_gridParams.height,-1);
}

void MultiLidarSlam::initConnections() {    
    _mapPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    auto robots = this->get_parameter("robot_namespaces").as_string_array();
    for(auto robot: robots)
        addNewRobot(robot);
}

void MultiLidarSlam::addNewRobot(const std::string &robotName) {
    auto pose_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + robotName + "/sim_ground_truth_pose", 10,
        [this, robotName](nav_msgs::msg::Odometry::SharedPtr msg) {
            _robots[robotName].latest_pose.pose = msg->pose.pose;
            _robots[robotName].hasPose = true;
        }
    );

    auto scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/" + robotName + "/lidar/scan", 10,
        [this, robotName](sensor_msgs::msg::LaserScan::SharedPtr msg) {
            //RCLCPP_INFO(get_logger(), "Before IF");
            if(!_robots[robotName].hasPose)
                return;
            elaborateScan(*msg, _robots[robotName].latest_pose);

        }
    );
    _scan_subs.push_back(scan_sub);
    _odom_subs.push_back(pose_sub);
    _robots[robotName] = RobotState();

}

void MultiLidarSlam::elaborateScan(const sensor_msgs::msg::LaserScan &msg, const geometry_msgs::msg::PoseStamped &poseMsg){

    const double px = poseMsg.pose.position.x;
    const double py = poseMsg.pose.position.y;
    tf2::Quaternion q(
        poseMsg.pose.orientation.x,
        poseMsg.pose.orientation.y,
        poseMsg.pose.orientation.z,
        poseMsg.pose.orientation.w);
    tf2::Matrix3x3 R(q);

    int gx0, gy0;
    if (!worldToMap(px, py, gx0, gy0)) return;
    double angle = msg.angle_min + M_PI_2;

    for (size_t i = 0; i < msg.ranges.size(); ++i, angle+=msg.angle_increment) {
        float r = msg.ranges[i];
        
        double lx = r*std::cos(angle);
        double ly = r*std::sin(angle);

        double mx = R[0][0]*lx + R[0][1]*ly + px;
        double my = R[1][0]*lx + R[1][1]*ly + py;

        int gx1, gy1;
        if (!worldToMap(mx, my, gx1, gy1)) continue;
        raytrace(gx0, gy0, gx1, gy1);
    }
    occupancy_grid.header.stamp = now();
    _mapPublisher->publish(occupancy_grid);

}

bool MultiLidarSlam::worldToMap(double wx, double wy, int &mx, int &my) {
    mx = static_cast<int>((wx-_gridParams.origin_x)/_gridParams.resolution);
    my = static_cast<int>((wy-_gridParams.origin_y)/_gridParams.resolution);
    return (mx>=0 && mx<_gridParams.width && my>=0 && my<_gridParams.height);
}
void MultiLidarSlam::raytrace(int x0, int y0, int x1, int y1)
{
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    int x = x0, y = y0;
    while (true) {
        size_t idx = y * _gridParams.width + x;
        occupancy_grid.data[idx] = 0; // free
        if (x == x1 && y == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }

    // Mark endpoint occupied
    size_t idx = y1 * _gridParams.width + x1;
    occupancy_grid.data[idx] = 100;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiLidarSlam>());
    rclcpp::shutdown();
    return 0;
}

#endif