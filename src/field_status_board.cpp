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
#include <rcll_vis_msgs/Machines.h>
#include <rcll_vis_msgs/Robots.h>
#include <rcll_vis_msgs/Products.h>
#include <rcll_vis_msgs/SetGameField.h>

#include <drawing.h>
#include <elements.h>

/* =====================================================================================================|
nodename:           "field_status_board"

This node draws the field status board for both teams
====================================================================================================== */

namespace {
    rcll_draw::AreaField main_area_field;
}

void cb_gameinfo(rcll_vis_msgs::GameInfo msg){
    main_area_field.setGameInfo(msg);
}

void cb_gamefield(rcll_vis_msgs::SetGameField msg){
    main_area_field.setGameField(msg);
}

void cb_machines(rcll_vis_msgs::Machines msg){
    main_area_field.setMachines(msg.machines);
}

void cb_robots(rcll_vis_msgs::Robots msg){
    main_area_field.setRobots(msg.robots);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "field_status_board");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loop_rate(4.0);
    int res_x = 1920;
    int res_y = 1080;
    bool fullscreen = false;
    bool refbox_view = false;
    std::string image_path = "";

    ros::Subscriber sub_gameinfo = nh.subscribe("refbox/gameinfo", 10, cb_gameinfo);
    ros::Subscriber sub_gamefield = nh.subscribe("refbox/gamefield", 10, cb_gamefield);
    ros::Subscriber sub_robots = nh.subscribe("refbox/robots", 10, cb_robots);
    ros::Subscriber sub_machines = nh.subscribe("refbox/machines", 10, cb_machines);

    private_nh.getParam("screen_x", res_x);
    private_nh.getParam("screen_y", res_y);
    private_nh.getParam("fullscreen", fullscreen);
    private_nh.getParam("image_path", image_path);
    private_nh.getParam("refbox_view", refbox_view);

    if (image_path == ""){
        ROS_ERROR("Image path must not be empty!");
        return 0;
    }

    std::string title = "FIELD STATUS BOARD";
    rcll_draw::setImagePath(image_path);

    int bordergapsize = 0.05 * res_y;

    cv::namedWindow(title, cv::WINDOW_NORMAL);

    if (fullscreen){
        cv::setWindowProperty(title, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    }

    cv::Mat mat(res_y, res_x, CV_8UC4);
    main_area_field.setGeometry(bordergapsize, bordergapsize, res_x - 2 * bordergapsize, res_y - 2 * bordergapsize);
    main_area_field.setRefBoxView(refbox_view);
    ros::spinOnce();

    while(ros::ok() && (cv::getWindowProperty(title.c_str(),cv::WND_PROP_AUTOSIZE)>=0)){
        loop_rate.sleep();
        cv::rectangle(mat, cv::Point(0,0), cv::Point(res_x, res_y), rcll_draw::getColor(rcll_draw::C_WHITE), cv::FILLED, 0);
        main_area_field.draw(mat, false);
        cv::imshow(title, mat);
        char key = (char)cv::waitKey(1);
        if (key == 27){
            cv::setWindowProperty(title, 0, cv::WINDOW_NORMAL);
        } else if (key == 70 || key == 102){
            cv::setWindowProperty(title, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        } else if (key == 81 || key == 113){
            return 0;
        }
        ros::spinOnce();
    }
    return 0;
}
