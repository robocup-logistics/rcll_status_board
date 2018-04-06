#include <ros/ros.h>

#include <llsf_communicator.h>

#include <rcll_msgs/GameInfo.h>
#include <rcll_msgs/MachinesStatus.h>
#include <rcll_msgs/Robots.h>
#include <rcll_msgs/Products.h>
#include <rcll_msgs/SetMachines.h>
#include <rcll_msgs/SetRobot.h>
#include <rcll_msgs/SetGameField.h>

/* =====================================================================================================|
nodename:           "static_values_publisher"

This node publishes static values and calls services once for testing purposes
====================================================================================================== */

//LLSFRefBoxCommunicator *refcomm_ptr = NULL;

int main(int argc, char** argv){
    ros::init(argc, argv, "llsf_interface");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2.0);

    LLSFRefBoxCommunicator refcomm;
    refcomm.run();
    //refcomm_ptr = &refcomm;

    ROS_INFO("Entering loop");
    while(ros::ok()){


        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
