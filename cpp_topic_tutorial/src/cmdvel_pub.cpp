#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "cmd_vel_pub_node");
    ros::NodeHandle nh;
    ROS_INFO("==== DriveForward node Started, move forward during 10 seconds ====\n");

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    ros::Rate r(5);

    geometry_msgs::Twist go_forward;
    go_forward.linear.x = 0.5;

    geometry_msgs::Twist stop;
    stop.linear.x = 0.0;

    ros::Time begin = ros::Time::now();

    while(ros::ok()) {

        ros::Time current = ros::Time::now();
        ros::Duration duration = current - begin;

        if (duration.sec >= 3.0){
            ROS_INFO(" 3 seconds left, Stop!! \n");
            pub.publish(stop);
            break;
        }

        pub.publish(go_forward);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}