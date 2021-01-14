// gazebo model spawn by rosservice
// referenced from answers.ros.org
// url : https://pastebin.com/UTWJSScZ

// basic template of roscpp service client
// referenced from wiki.ros.org
// url : http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>

void addXml(gazebo_msgs::SpawnModel& model_in, const std::string& file_path ){
    std::ifstream file(file_path);
    std::string line;

    while (!file.eof()){
        std::getline(file, line);
        model_in.request.model_xml += line;
    }
    file.close();
}

geometry_msgs::Pose getPose(){
    geometry_msgs::Pose initial_pose;

    initial_pose.position.x = -2;
    initial_pose.position.y = 1;
    initial_pose.position.z = 1;

    initial_pose.orientation.z = -0.707;
    initial_pose.orientation.w = 0.707;

    return initial_pose;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "gazebo_spawn_model");

    ros::NodeHandle nh;
    ros::ServiceClient spawn_model_prox = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_urdf_model");
    
    gazebo_msgs::SpawnModel model;

    // add roslib in find_package()
    auto file_path = ros::package::getPath("cpp_service_tutorial") +  "/models/r2d2.urdf";

    addXml(model, file_path);

    model.request.model_name = "r2d2";
    model.request.reference_frame = "world";

    model.request.initial_pose = getPose();

    // ServiceClient.call() => return bool type
    if (spawn_model_prox.call(model)){
        auto response = model.response;
        ROS_INFO("%s", response.status_message.c_str()); // Print the result given by the service called
    }
    else {
        ROS_ERROR("Failed to call service /trajectory_by_name");
        return 1;   
    }

    return 0;
}
