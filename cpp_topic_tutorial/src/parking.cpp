#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class TinyBot
{
private:
    ros::NodeHandle m_nh;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;

    geometry_msgs::Twist m_cmd_vel;

public:
    TinyBot(){
        ROS_INFO("Publisher and Subscriber initialized");
        m_pub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
        m_sub = m_nh.subscribe("/scan", 1, &TinyBot::subCallback, this);
    }

    void robotStop(){
        m_cmd_vel.linear.x = 0.0;
        m_pub.publish(m_cmd_vel);
    }

    void robotGo(){
        m_cmd_vel.linear.x = 0.8;
        m_pub.publish(m_cmd_vel);
    }

    void subCallback(const sensor_msgs::LaserScan &data){

        auto frontLidarPoint = data.ranges[360];
        std::cout << frontLidarPoint << std::endl;

        if ( frontLidarPoint >= 1.0 ){
            this->robotGo();
        }
        else {
            this->robotStop();
        }

        ROS_INFO("===== front lidar point val : %f", frontLidarPoint);
    }
};

int main(int argv, char** argc) {

    ros::init(argv, argc, "parking_node");
    TinyBot tinybot;
    ros::spin();

    return 0;
}