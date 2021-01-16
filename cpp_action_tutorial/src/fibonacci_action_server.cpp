/*
 * referenced from wiki.ros.org 
 * url : http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29 
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>

/* Fibonacci.action
* 
* goal definition
* int32 order
* ---
* result definition
* int32[] sequence
* ---
* feedback
* int32[] sequence
*/ 

class FibonacciActionClass {
protected:
    ros::NodeHandle m_nh;
    
    actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> m_as;
    std::string m_name;

    // vector 
    actionlib_tutorials::FibonacciFeedback m_feedback;
    actionlib_tutorials::FibonacciResult m_result;

public:
    FibonacciActionClass(const std::string &name_in = "fibonacci_action_server"): 
        m_name(name_in), 
        m_as(m_nh, name_in, boost::bind(&FibonacciActionClass::actionCallback, this, _1), false) {
        m_as.start();
        ROS_INFO("==== Action Server Class Constructed ====");
    }
    
    ~FibonacciActionClass(){
        ROS_INFO("==== Action Server Class Destroying ====");
    }

    // must use Ptr type or ConstPtr type!!
    void actionCallback(const actionlib_tutorials::FibonacciGoalConstPtr &goal){

        ros::Rate r(5);
        bool success = true;
        int order = int(goal->order);
        const auto sequence = &(m_feedback.sequence);

        // push_back the seeds for the fibonacci sequence
        sequence->clear();
        sequence->push_back(0);
        sequence->push_back(1);

        // publish info to the console for the user
        std::cout << m_name.c_str() << ": Executing, creating fibonacci sequence of order " << 
            order << " with seeds " << sequence->at(0) << " , " << sequence->at(1) << std::endl;

        // start executing the action
        for(int i = 1; i <= order; i++){
            // check that preempt has not been requested by the client
            if (m_as.isPreemptRequested() || !ros::ok()){
                ROS_INFO("%s: Preempted", m_name.c_str());
                // set the action state to preempted
                m_as.setPreempted();
                success = false;
                break;
            }
            sequence->push_back(sequence->at(i) + sequence->at(i-1));
            // publish the feedback
            m_as.publishFeedback(m_feedback);
            // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep();
        }

        if(success){
            m_result.sequence = m_feedback.sequence;
            ROS_INFO("%s: Succeeded", m_name.c_str());
            // set the action state to succeeded
            m_as.setSucceeded(m_result);
        }
    }
};

int main(int argc, char** argv){

    ros::init(argc, argv, "fibonacci");
    FibonacciActionClass fibonacci;
    ros::spin();

    return 0;
}