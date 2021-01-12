#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void sub_callback(const sensor_msgs::LaserScan &data ){
    std::cout << "Front Lidar Value : " << data.ranges[360] << std::endl;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "laser_sub_node");
    ros::NodeHandle nh;    
    ros::Subscriber sub = nh.subscribe("/scan", 1, sub_callback);

    ros::spin();

    return 0;
}