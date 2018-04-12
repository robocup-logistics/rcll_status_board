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

    double field_length = 1.0;
    double field_width = 1.0;
    int zones_x = 14;
    int zones_y = 8;
    std::vector<float> walls;
    std::vector<int> insertion_zones;

    nh.getParam("/rcll/gamefield/field_length", field_length);
    nh.getParam("/rcll/gamefield/field_width", field_width);
    nh.getParam("/rcll/gamefield/zones_x", zones_x);
    nh.getParam("/rcll/gamefield/zones_y", zones_y);
    nh.getParam("/rcll/gamefield/walls", walls);
    nh.getParam("/rcll/gamefield/insertion_zones", insertion_zones);

    ros::Duration(1.0).sleep();

    rcll_msgs::SetGameField gamefield_msg;
    gamefield_msg.walls = walls;
    gamefield_msg.insertion_zones = insertion_zones;
    gamefield_msg.field_length = field_length;
    gamefield_msg.field_width = field_width;
    gamefield_msg.zones_x = zones_x;
    gamefield_msg.zones_y = zones_y;
    pub_setgamefield.publish(gamefield_msg);

    LLSFRefBoxCommunicator refcomm;
    refcomm.run();
    return 0;
}
