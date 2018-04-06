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

int main(int argc, char** argv){
    ros::init(argc, argv, "llsf_interface");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2.0);

    ros::Publisher pub_robots = nh.advertise<rcll_msgs::Robots>("refbox/update_robots", 10);
    ros::Publisher pub_setgamefield = nh.advertise<rcll_msgs::SetGameField>("refbox/set_gamefield", 10);
    ros::Publisher pub_setrobot = nh.advertise<rcll_msgs::SetRobot>("refbox/set_robot", 10);
    ros::Publisher pub_machinesstatus = nh.advertise<rcll_msgs::MachinesStatus>("refbox/update_machines", 10);
    ros::Publisher pub_products = nh.advertise<rcll_msgs::Products>("refbox/update_products", 10);

    ros::Duration(1.0).sleep();

    std::vector<float> walls = {
        -7.0, 8.0, 7.0, 8.0,
        -7.0, 6.5, -7.0, 8.0,
        7.0, 6.5, 7.0, 8.0,
        -7.0, 1.0, -7.0, 2.0,
        7.0, 1.0, 7.0, 2.0,
        -7.0, 1.0, -5.0, 1.0,
        5.0, 1.0, 7.0, 1.0,
        -7.0, 0.0, -4.0, 0.0,
        4.0, 0.0, 7.0, 0.0,
        -4.0, 0.0, -4.0, 1.0,
        4.0, 0.0, 4.0, 1.0,
        -2.0, 0.0, 2.0, 0.0
    };

    std::vector<int> insertion_zones = {51, 61, 71};

    rcll_msgs::SetGameField gamefield_msg;
    gamefield_msg.walls = walls;
    gamefield_msg.insertion_zones = insertion_zones;
    gamefield_msg.field_w = 14.0;
    gamefield_msg.field_h = 8.0;
    gamefield_msg.zones_x = 14;
    gamefield_msg.zones_y = 8;
    pub_setgamefield.publish(gamefield_msg);

    LLSFRefBoxCommunicator refcomm;
    refcomm.run();
    return 0;
}
