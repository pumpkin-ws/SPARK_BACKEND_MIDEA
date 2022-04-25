#include <ros/ros.h>
#include "actionlib/server/simple_action_server.h"
#include "spark_backend/RobotServiceAction.h"
#include <thread>


class TestServer{

protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<spark_backend::RobotServiceAction> as;
    spark_backend::RobotServiceFeedback feedback;
    spark_backend::RobotServiceResult result;
    std::string action_name;
    int m_goal;
    
public:

    // the constructor
    TestServer(std::string name) : 
        as(nh, name, false) {
            as.registerGoalCallback(boost::bind(&TestServer::goalCB, this));
            as.registerPreemptCallback(boost::bind(&TestServer::preemptCB, this));
            as.start();
        }
    // the destructor
    ~TestServer(void) {

    }
    
    // the goal callback function 
    void goalCB() {
        if(as.isNewGoalAvailable()) {
            ROS_INFO("New goal is available!");
            auto new_goal = as.acceptNewGoal();
            m_goal = new_goal->goal;
        }

        // generate different threads depending on the type of goal received
        // use a thread to break the goal initialization 
        std::thread t(&TestServer::print, this);
        t.detach(); // detach the thread from the goal initialization function block
    }
    
    void preemptCB() {
        ROS_INFO("Preempt goal now.");
        as.setPreempted();
    }

    void print() {
        // abort pursuit of the goal if the number is irresonable
        if(m_goal < 0) {
            ROS_INFO("Abort the program!");
            as.setAborted();
        }
        
        for(int i = 0; i < m_goal; i++) {
            if(as.isActive()) { // check if the server is set to be inactive before executing other commands
                std::cout << i << std::endl;
                sleep(1);
                feedback.data = i;
                as.publishFeedback(feedback);
            } else {
                ROS_INFO("The goal is preempted by the client!");
                break;
                return; // return here will not break out of the function. The following lines will still
            }        
        }
        result.has_arrived = true;
        as.setSucceeded(result);
        ROS_INFO("Execution Succeeded");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "server");
    TestServer ts("simple");    
    ros::spin();
    
    return EXIT_SUCCESS;
}