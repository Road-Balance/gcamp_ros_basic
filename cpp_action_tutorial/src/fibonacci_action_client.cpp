// referenced from wiki.ros.org 
// url : http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_tutorials/FibonacciAction.h>

// Fibonacci.action
// 
// #goal definition
// int32 order
// ---
// #result definition
// int32[] sequence
// ---
// #feedback
// int32[] sequence

void doneCb(const actionlib::SimpleClientGoalState& state,
            const actionlib_tutorials::FibonacciResultConstPtr& result)
{
    ROS_INFO("[State Result]: %s", state.toString().c_str());
    ROS_INFO("Answer: %i", result->sequence.back());
    ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
    ROS_INFO("Goal just went active");
}

void feedbackCb(const actionlib_tutorials::FibonacciFeedbackConstPtr& feedback){
    ROS_INFO("[Feedback] :");
    for(auto &elem : feedback->sequence){
        std::cout << elem << " ";
    }
    std::cout << std::endl;
}

int main(int argv, char** argc){

    ros::init(argv, argc, "fibonacci_client");
    ros::NodeHandle nh;
    ros::Rate r(5);

    std::string server_name = "fibonacci_action_server";
    // turn on auto threading
    actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> client(server_name, true); 
    std::cout << "Wating for Server..." << std::endl;
    client.waitForServer();

    actionlib_tutorials::FibonacciGoal goal;
    goal.order = 20;

    std::cout << "==== Sending Goal to Server ====" << std::endl;
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    actionlib::SimpleClientGoalState state_result = client.getState();

    while (!state_result.isDone()){
        // Doing Stuff while waiting for the Server to give a result....

        // if (<cancel-condition>){
        //     client.cancelGoal();
        // }

        state_result = client.getState();
        r.sleep();
    }

    if (state_result == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("[Action Done State Result]: %s", state_result.toString().c_str());
    else 
        ROS_ERROR("[Something Gonna Wrong State Result]: %s", state_result.toString().c_str());

    return 0;
}