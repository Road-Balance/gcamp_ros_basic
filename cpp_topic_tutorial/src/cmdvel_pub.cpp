/*
 * basic topic publisher example
 * 
 * referenced from wiki.ros.org : http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
 */

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

    double timedelta;
    clock_t start = clock();
    clock_t end = clock();

    timedelta = (double)(end - start) / CLOCKS_PER_SEC;

    while (timedelta < 10.0){
        cmd_pub.publish(go_forward);
        
        end = clock();
        timedelta = (double)(end - start) / CLOCKS_PER_SEC;
    }
    
    ROS_WARN(" 10 seconds passed, Stop!! \n");
    cmd_pub.publish(stop);

    return 0;
}