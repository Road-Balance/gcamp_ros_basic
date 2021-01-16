/*
 * Park your robot with lidar distance value
 * 
 * created by kimsooyoung : https://github.com/kimsooyoung
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class TinyBot
{
private:
    ros::NodeHandle m_nh;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;

    std::string m_name;
    geometry_msgs::Twist m_cmd_vel;

public:
    TinyBot(const std::string &name_in = "my_tiny"): m_name(name_in) {
        ROS_INFO("Publisher and Subscriber initialized");
        m_pub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
        m_sub = m_nh.subscribe("/scan", 1, &TinyBot::subCallback, this);
    }

    void robotStop(){
        m_cmd_vel.linear.x = 0.0;
        m_pub.publish(m_cmd_vel);
        ROS_INFO(" Parking Done!!"); 
        ros::shutdown();
    }

    void robotGo(){
        m_cmd_vel.linear.x = 0.8;
        m_pub.publish(m_cmd_vel);
    }

    void subCallback(const sensor_msgs::LaserScan &data){

        auto frontLidarPoint = data.ranges[360];

        if ( frontLidarPoint >= 1.0 ){
            this->robotGo();
        }
        else {
            this->robotStop();
        }

        std::cout << "===== front lidar point val : " << frontLidarPoint << " ====" << std::endl;
    }
};

int main(int argv, char** argc) {

    ros::init(argv, argc, "parking_node");
    TinyBot tinybot("gcamp_robo");
    ros::spin();

    return 0;
}