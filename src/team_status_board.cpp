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
#include <rcll_vis_msgs/Robots.h>
#include <rcll_vis_msgs/Products.h>
#include <rcll_vis_msgs/Machines.h>

#include <drawing.h>
#include <elements.h>

/* =====================================================================================================|
nodename:           "team_status_board"

This node draws the team status board for a given teamcolor
====================================================================================================== */

namespace {
    rcll_draw::GamePhase gamephase;

    rcll_draw::Team team = rcll_draw::NO_TEAM;

    rcll_draw::AreaPreGameSetup area_pregamesetup;
    rcll_draw::AreaExplorationTeam area_exploration;
    rcll_draw::AreaProductionTeam area_production;
    rcll_draw::AreaPostGame area_postgame;
}

void cb_gameinfo(rcll_vis_msgs::GameInfo msg){
    gamephase = (rcll_draw::GamePhase)msg.game_phase;
    area_pregamesetup.setGameInfo(msg);
    area_exploration.setGameInfo(msg);
    area_production.setGameInfo(msg);
    area_postgame.setGameInfo(msg);
}

void cb_machines(rcll_vis_msgs::Machines msg){
    area_exploration.setMachines(msg.machines);
    area_production.setMachines(msg.machines);
}

void cb_robots(rcll_vis_msgs::Robots msg){
    area_production.setRobots(msg.robots);
}

void cb_products(const rcll_vis_msgs::Products::ConstPtr& msg){
    area_production.setProductsCount(msg->orders.size());
    for (size_t i = 0; i < msg->orders.size(); i++){
        rcll_draw::ProductInformation pi;
        pi.product_id = msg->orders[i].product_id;
        pi.quantity_id = msg->orders[i].quantity_id;
        pi.deadline = msg->orders[i].end_delivery_time;
        pi.points_max = msg->orders[i].points_max;
        pi.plan.complexity = msg->orders[i].complexity;
        pi.plan.base = msg->orders[i].structure[0];
        pi.plan.ring1 = msg->orders[i].structure[1];
        pi.plan.ring2 = msg->orders[i].structure[2];
        pi.plan.ring3 = msg->orders[i].structure[3];
        pi.plan.cap = msg->orders[i].structure[4];
        if (team == rcll_draw::CYAN){
            pi.plan.status_base = msg->orders[i].step_stati_cyan[0];
            pi.plan.status_ring1 = msg->orders[i].step_stati_cyan[1];
            pi.plan.status_ring2 = msg->orders[i].step_stati_cyan[2];
            pi.plan.status_ring3 = msg->orders[i].step_stati_cyan[3];
            pi.plan.status_cap = msg->orders[i].step_stati_cyan[4];
            pi.plan.status_product = msg->orders[i].step_stati_cyan[5];

            pi.progress = msg->orders[i].progress_cyan;
            pi.points = msg->orders[i].points_cyan;
        } else if (team == rcll_draw::MAGENTA){
            pi.plan.status_base = msg->orders[i].step_stati_magenta[0];
            pi.plan.status_ring1 = msg->orders[i].step_stati_magenta[1];
            pi.plan.status_ring2 = msg->orders[i].step_stati_magenta[2];
            pi.plan.status_ring3 = msg->orders[i].step_stati_magenta[3];
            pi.plan.status_cap = msg->orders[i].step_stati_magenta[4];
            pi.plan.status_product = msg->orders[i].step_stati_magenta[5];

            pi.progress = msg->orders[i].progress_magenta;
            pi.points = msg->orders[i].points_magenta;
        }

        area_production.setProduct(pi, i);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "team_status_board");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loop_rate(4.0);
    int team_int = -1;
    int res_x = 1920;
    int res_y = 1080;
    bool fullscreen = false;
    double product_page_time = 10.0;
    std::string image_path = "";

    ros::Subscriber sub_gameinfo = nh.subscribe("refbox/gameinfo", 10, cb_gameinfo);
    ros::Subscriber sub_robots = nh.subscribe("refbox/robots", 10, cb_robots);
    ros::Subscriber sub_machines = nh.subscribe("refbox/machines", 10, cb_machines);
    ros::Subscriber sub_products = nh.subscribe("refbox/products", 10, cb_products);

    private_nh.getParam("side", team_int);
    private_nh.getParam("screen_x", res_x);
    private_nh.getParam("screen_y", res_y);
    private_nh.getParam("fullscreen", fullscreen);
    private_nh.getParam("image_path", image_path);
    private_nh.getParam("product_page_time", product_page_time);

    if (image_path == ""){
        ROS_ERROR("Image path must not be empty!");
        return 0;
    }

    std::string title;
    rcll_draw::setImagePath(image_path);

    if (team_int == rcll_draw::CYAN){
        title = "STATUS BOARD - CYAN";
        team = rcll_draw::CYAN;
    } else if (team_int == rcll_draw::MAGENTA){
        title = "STATUS BOARD - MAGENTA";
        team = rcll_draw::MAGENTA;
    } else {
        title = "STATUS BOARD";
    }

    int bordergapsize = 0.05 * res_y;
    int gapsize = 0.02 * res_y;

    cv::namedWindow(title, CV_WINDOW_NORMAL);

    if (fullscreen){
        cv::setWindowProperty(title, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    }

    cv::Mat mat(res_y, res_x, CV_8UC4);
    cv::rectangle(mat, cv::Point(0,0), cv::Point(res_x, res_y), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED, 0);

    area_pregamesetup = rcll_draw::AreaPreGameSetup(team);
    area_pregamesetup.setGeometry(bordergapsize, bordergapsize, res_x - 2 * bordergapsize, res_y - 2 * bordergapsize);

    area_exploration = rcll_draw::AreaExplorationTeam(team);
    area_exploration.setGeometry(bordergapsize, bordergapsize, res_x - 2 * bordergapsize, res_y - 2 * bordergapsize);

    area_production = rcll_draw::AreaProductionTeam(team);
    area_production.setGeometry(bordergapsize, bordergapsize, res_x - 2 * bordergapsize, res_y - 2 * bordergapsize, gapsize);

    area_postgame = rcll_draw::AreaPostGame(team);
    area_postgame.setGeometry(bordergapsize, bordergapsize, res_x - 2 * bordergapsize, res_y - 2 * bordergapsize);

    ros::spinOnce();

    double paging_timer = product_page_time;
    while(ros::ok() && cvGetWindowHandle(title.c_str())){
        loop_rate.sleep();
        cv::rectangle(mat, cv::Point(0,0), cv::Point(res_x, res_y), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED, 0);

        if(gamephase == rcll_draw::PRE_GAME){
            area_pregamesetup.draw(mat, false);
        } else if(gamephase == rcll_draw::SETUP){
            area_pregamesetup.draw(mat, false);
        } else if (gamephase == rcll_draw::EXPLORATION){
            area_exploration.draw(mat, false);
        } else if (gamephase == rcll_draw::PRODUCTION){

            paging_timer +=loop_rate.expectedCycleTime().toSec();

            if (paging_timer >= product_page_time){
                area_production.paging();
                paging_timer = 0.0;
            }

            area_production.draw(mat, false);
        } else if (gamephase == rcll_draw::POST_GAME){
            area_postgame.draw(mat, false);
        }

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
