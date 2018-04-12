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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <rcll_vis_msgs/GameInfo.h>
#include <rcll_vis_msgs/MachinesStatus.h>
#include <rcll_vis_msgs/Robots.h>
#include <rcll_vis_msgs/Products.h>
#include <rcll_vis_msgs/SetMachines.h>
#include <rcll_vis_msgs/SetRobot.h>
#include <rcll_vis_msgs/SetGameField.h>

#include <drawing.h>
#include <elements.h>

/* =====================================================================================================|
nodename:           "team_status_board"

This node draws the team status board for a given teamcolor
====================================================================================================== */

namespace {
    std::map<int, std::string> gamestates;
    std::map<int, std::string> gamephases;

    rcll_draw::FieldArea main_area_field;
}

void cb_gameinfo(const rcll_vis_msgs::GameInfo::ConstPtr& msg){
    //ROS_INFO("Updating gameinfo");
    main_area_field.setTeam(msg->team_name_cyan, rcll_draw::CYAN);
    main_area_field.setTeam(msg->team_name_magenta, rcll_draw::MAGENTA);
    main_area_field.setGameInfo(gamestates[msg->game_state], gamephases[msg->game_phase], (int)msg->phase_time, msg->team_points_cyan, msg->team_points_magenta);
}

void cb_gamefield(const rcll_vis_msgs::SetGameField::ConstPtr& msg){
    ROS_INFO("Initializing gamefield with w=%f h=%f zx=%i zy=%i", msg->field_length, msg->field_width, msg->zones_x, msg->zones_y);
    main_area_field.setLayout(msg->field_length, msg->field_width, msg->zones_x, msg->zones_y, msg->insertion_zones);
    main_area_field.setWalls(msg->walls);
}

void cb_set_machine(const rcll_vis_msgs::SetMachines::ConstPtr& msg){
    ROS_INFO("Initializing machines");
    for (size_t i = 0; i < msg->machines.size(); i++){
        ROS_INFO("  name=%s team=%i index=%i", msg->machines[i].name_short.c_str(), msg->machines[i].team, msg->machines[i].index);
        main_area_field.setMachine(msg->machines[i].name_short, (rcll_draw::Team)msg->machines[i].team, msg->machines[i].index);
        main_area_field.setMachinePos(msg->machines[i].x, msg->machines[i].y, msg->machines[i].yaw, msg->machines[i].index);
    }
}

void cb_machines(const rcll_vis_msgs::MachinesStatus::ConstPtr& msg){
    for (size_t i = 0; i < msg->machines.size(); i++){
        main_area_field.setMachineReport(msg->machines[i].machine_status_exploration1, msg->machines[i].machine_status_exploration2, msg->machines[i].index);
    }
}

void cb_set_robot(const rcll_vis_msgs::SetRobot::ConstPtr& msg){
    ROS_INFO("Initializing robot name=%s number=%i team=%i, index=%i", msg->robot_name.c_str(), msg->robot_id, msg->team, msg->index);
    main_area_field.addRobot(msg->robot_name, msg->robot_id, (rcll_draw::Team)msg->team);
}

void cb_robots(const rcll_vis_msgs::Robots::ConstPtr& msg){
    for (size_t i = 0; i < msg->robots.size(); i++){
        main_area_field.setRobotPos(msg->robots[i].x, msg->robots[i].y, msg->robots[i].yaw, msg->robots[i].index);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "field_status_board");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loop_rate(4.0);
    int res_x = 1920;
    int res_y = 1080;
    bool fullscreen = false;
    std::string image_path = "";

    ros::Subscriber sub_gameinfo = nh.subscribe("refbox/gameinfo", 10, cb_gameinfo);
    ros::Subscriber sub_gamefield = nh.subscribe("refbox/set_gamefield", 10, cb_gamefield);
    ros::Subscriber sub_addrobot = nh.subscribe("refbox/set_robot", 10, cb_set_robot);
    ros::Subscriber sub_robots = nh.subscribe("refbox/update_robots", 10, cb_robots);
    ros::Subscriber sub_addmachine = nh.subscribe("refbox/set_machines", 10, cb_set_machine);
    ros::Subscriber sub_machines = nh.subscribe("refbox/update_machines", 10, cb_machines);

    private_nh.getParam("screen_x", res_x);
    private_nh.getParam("screen_y", res_y);
    private_nh.getParam("fullscreen", fullscreen);
    private_nh.getParam("image_path", image_path);

    if (image_path == ""){
        ROS_ERROR("Image path must not be empty!");
        return 0;
    }

    gamestates[0] = "INIT";
    gamestates[1] = "WAIT START";
    gamestates[2] = "RUNNING";
    gamestates[3] = "PAUSED";

    gamephases[0] = "PRE GAME";
    gamephases[10] = "SETUP";
    gamephases[20] = "EXPLORATION";
    gamephases[30] = "PRODUCTION";
    gamephases[40] = "POST GAME";

    rcll_draw::Team team = rcll_draw::NO_TEAM;
    std::string title = "FIELD STATUS BOARD";
    rcll_draw::setImagePath(image_path);

    int bordergapsize = 0.05 * res_y;
    int gapsize = 0.02 * res_y;

    cv::namedWindow(title, CV_WINDOW_NORMAL);

    if (fullscreen){
        cv::setWindowProperty(title, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    }

    cv::Mat mat(res_y, res_x, CV_8UC4);
    rcll_draw::HeaderPanel header(title, team);
    header.setGeometry(bordergapsize, res_x, bordergapsize);
    main_area_field.setGeometry(bordergapsize, bordergapsize * 3, res_x - 2 * bordergapsize, res_y - 4 * bordergapsize, gapsize);

    while(ros::ok() && cvGetWindowHandle(title.c_str())){
        loop_rate.sleep();
        cv::rectangle(mat, cv::Point(0,0), cv::Point(res_x, res_y), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED, 0);
        header.draw(mat);
        main_area_field.draw(mat);
        cv::imshow(title, mat);
        char key = (char)cv::waitKey(1);
        if (key == 27){
            cv::setWindowProperty(title, 0, CV_WINDOW_NORMAL);
        } else if (key == 70 || key == 102){
            cv::setWindowProperty(title, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
        } else if (key == 81 || key == 113){
            return 0;
        }
        ros::spinOnce();
    }
    return 0;
}
