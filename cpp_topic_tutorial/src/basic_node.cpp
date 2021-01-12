#include <ros/ros.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "basic_node");
    ros::NodeHandle nh;
    ros::Rate r(5);

    while ( ros::ok() ) {
        ROS_INFO("This is Basic Node");
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
