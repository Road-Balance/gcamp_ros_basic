// turn your robot with angular velocity
// created by kimsooyoung : https://github.com/kimsooyoung

#include <ros/ros.h>
#include "cpp_service_tutorial/ControlTurningMessage.h"

typedef cpp_service_tutorial::ControlTurningMessage srv_t;

using std::cout;
using std::endl;
using std::cin;

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_turning_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<srv_t>("/control_robot_angle");

    ROS_INFO("==== Service Client initialized ====");

    // srv type

    // uint32 time_duration
    // float64 angular_vel
    // ---
    // bool success

    srv_t srv;
    
    unsigned int time_duration;
    float angular_vel;

    cout << "Enter time_duration : ";
    cin >> time_duration;
    cin.ignore(32767, '\n'); cin.clear();

    cout << "Enter angular_vel : ";
    cin >> angular_vel;
    cin.ignore(32767, '\n'); cin.clear();

    srv.request.time_duration = time_duration;
    srv.request.angular_vel = angular_vel;

    if ( client.call(srv) ){
        auto response = srv.response;
        cout << std::boolalpha;
        cout << "Response : " << bool(response.success) << endl;
    }
    else {
        ROS_ERROR("Failed to call service /control_robot_angle");
        return 1;      
    }

    return 0;
}
