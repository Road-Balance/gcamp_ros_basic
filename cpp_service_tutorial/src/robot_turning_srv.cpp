// referenced from answer.ros.org
// url : https://answers.ros.org/question/214597/service-with-class-method/

#include <ros/ros.h>
#include <string>
#include <time.h> 
#include <geometry_msgs/Twist.h>
#include "cpp_service_tutorial/ControlTurningMessage.h"

typedef cpp_service_tutorial::ControlTurningMessage::Request Request_T;
typedef cpp_service_tutorial::ControlTurningMessage::Response Response_T;


class TinyBot {
private:
    ros::NodeHandle m_nh;
    ros::ServiceServer m_control_ss; // control service_server
    ros::Publisher m_vel_pub;

    geometry_msgs::Twist m_cmd_vel;
    std::string m_name;

public:
    TinyBot(const std::string &name_in = "tinybot"): m_name(name_in) {

        m_control_ss = m_nh.advertiseService("/control_robot_angle", &TinyBot::servCallback, this);
        m_vel_pub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
        ROS_INFO("Service Server and Publisher initialized");
        ROS_INFO("Waiting for request...");
    }

    // srv type

    // uint32 time_duration
    // float64 angular_vel
    // ---
    // bool success
    bool servCallback(Request_T &req, Response_T &res) {
        
        m_cmd_vel.angular.z = req.angular_vel;

        ros::Time begin_time = ros::Time::now();
        std::cout << int(req.time_duration) << std::endl;
        ros::Duration duration = ros::Duration(int(req.time_duration));

        ROS_INFO("==== Start Turning ====");
        m_vel_pub.publish(m_cmd_vel);
        duration.sleep();
        // while ( ros::Time::now() < end_time ){
        //     // std::cout << ros::Time::now() << std::endl;
        //     m_vel_pub.publish(m_cmd_vel);
        // }

        ROS_WARN("==== Stop Turning ====");
        m_cmd_vel.angular.z = 0;
        m_vel_pub.publish(m_cmd_vel);
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "robot_turning_server");
    auto myTinyBot = TinyBot();
    ros::spin();

    return 0;
}