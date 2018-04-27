/*
The MIT License (MIT)

Copyright (c) 2017-2018 Florian Eith <florian.eith@web.de>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <ros/ros.h>

#include <llsf_communicator.h>

#include <rcll_vis_msgs/GameInfo.h>
#include <rcll_vis_msgs/MachinesStatus.h>
#include <rcll_vis_msgs/Robots.h>
#include <rcll_vis_msgs/Products.h>
#include <rcll_vis_msgs/SetMachines.h>
#include <rcll_vis_msgs/SetRobot.h>
#include <rcll_vis_msgs/SetGameField.h>

/* =====================================================================================================|
nodename:           "static_values_publisher"

This node publishes static values and calls services once for testing purposes
====================================================================================================== */

int main(int argc, char** argv){
    ros::init(argc, argv, "llsf_interface");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loop_rate(2.0);

    ros::Publisher pub_robots = nh.advertise<rcll_vis_msgs::Robots>("refbox/update_robots", 10);
    ros::Publisher pub_setgamefield = nh.advertise<rcll_vis_msgs::SetGameField>("refbox/set_gamefield", 10, true);
    ros::Publisher pub_setrobot = nh.advertise<rcll_vis_msgs::SetRobot>("refbox/set_robot", 10, true);
    ros::Publisher pub_machinesstatus = nh.advertise<rcll_vis_msgs::MachinesStatus>("refbox/update_machines", 10);
    ros::Publisher pub_products = nh.advertise<rcll_vis_msgs::Products>("refbox/update_products", 10);

    double field_length = 1.0;
    double field_width = 1.0;
    int zones_x = 14;
    int zones_y = 8;
    std::vector<float> walls;
    std::vector<int> insertion_zones;
    bool refbox_remote = false;
    std::string refbox_host = "localhost";
    int refbox_recv_port = 4444;

    private_nh.getParam("refbox_remote", refbox_remote);
    if (refbox_remote){
        private_nh.getParam("refbox_host", refbox_host);
        private_nh.getParam("refbox_port_public_recv", refbox_recv_port);
        ROS_INFO("Refbox is remote at %s:%i", refbox_host.c_str(), refbox_recv_port);
    } else {
        ROS_INFO("Refbox is nearby at %s:%i", refbox_host.c_str(), refbox_recv_port);
    }

    nh.getParam("/rcll/gamefield/field_length", field_length);
    nh.getParam("/rcll/gamefield/field_width", field_width);
    nh.getParam("/rcll/gamefield/zones_x", zones_x);
    nh.getParam("/rcll/gamefield/zones_y", zones_y);
    nh.getParam("/rcll/gamefield/walls", walls);
    nh.getParam("/rcll/gamefield/insertion_zones", insertion_zones);

    ros::Duration(1.0).sleep();

    rcll_vis_msgs::SetGameField gamefield_msg;
    gamefield_msg.walls = walls;
    gamefield_msg.insertion_zones = insertion_zones;
    gamefield_msg.field_length = field_length;
    gamefield_msg.field_width = field_width;
    gamefield_msg.zones_x = zones_x;
    gamefield_msg.zones_y = zones_y;
    pub_setgamefield.publish(gamefield_msg);

    LLSFRefBoxCommunicator refcomm(refbox_host, refbox_recv_port);
    refcomm.run();
    return 0;
}
