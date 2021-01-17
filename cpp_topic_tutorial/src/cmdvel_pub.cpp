/*
 * basic topic publisher example
 * 
 * referenced from wiki.ros.org : http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
 */

#include <chrono>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "cmd_vel_pub_node");
    ros::NodeHandle nh;
    ROS_INFO("==== DriveForward node Started, move forward during 10 seconds ====\n");

    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    ros::Rate r(5);

    geometry_msgs::Twist go_forward;
    go_forward.linear.x = 0.5;

    geometry_msgs::Twist stop;
    stop.linear.x = 0.0;

    auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();

    std::chrono::duration<double> time_duration = now - start;


    while (time_duration.count() < 5.0){
        cmd_pub.publish(go_forward);
        
        now = std::chrono::steady_clock::now();
        time_duration = now - start;
    }
    
    ROS_WARN(" 5 seconds passed, Stop!! \n");
    cmd_pub.publish(stop);

    return 0;
}