#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

static geometry_msgs::Twist go_forward;
static geometry_msgs::Twist stop;
go_forward.linear.x = 0.3;
stop.linear.x = 0.3;

void sub_callback(const sensor_msgs::LaserScan &data){
    auto frontLidarPoint = data.ranges[360];

    if ( frontLidarPoint >= 0.3 ){
        pub.publish(go_forward);
    }
    else {
        pub.publish(stop);
    }

    ROS_INFO("===== front lidar point val : %d", frontLidarPoint);
}

int main(int argv, char** argc) {

    ros::init(argv, argc, "parking_node");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    ros::Subscriber sub = nh.subscribe("/scan", 1, sub_callback);

    
    ros::Rate r(5);
    
    ros::spin();

    return 0;
}