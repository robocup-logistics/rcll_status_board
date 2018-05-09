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

#include <rcll_vis_msgs/GameInfo.h>
#include <rcll_vis_msgs/Products.h>
#include <rcll_vis_msgs/Machines.h>
#include <rcll_vis_msgs/Robots.h>
#include <rcll_vis_msgs/SetGameField.h>

/* =====================================================================================================|
nodename:           "static_values_publisher"

This node publishes static values and calls services once for testing purposes
====================================================================================================== */

int main(int argc, char** argv){
    ros::init(argc, argv, "static_values_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2.0);

    ros::Publisher pub_gameinfo = nh.advertise<rcll_vis_msgs::GameInfo>("refbox/gameinfo", 10, true);
    ros::Publisher pub_robots = nh.advertise<rcll_vis_msgs::Robots>("refbox/robots", 10, true);
    ros::Publisher pub_gamefield = nh.advertise<rcll_vis_msgs::SetGameField>("refbox/gamefield", 10, true);
    ros::Publisher pub_machines = nh.advertise<rcll_vis_msgs::Machines>("refbox/machines", 10, true);
    ros::Publisher pub_products = nh.advertise<rcll_vis_msgs::Products>("refbox/products", 10, true);

    ros::Duration(1.0).sleep();

    std::vector<std::string> machine_statuses = {"IDLE", "BROKEN", "PROCESSING", "PROCESSED", "PREPARED", "DOWN", "READY-AT-OUTPUT", "WAIT-IDLE", "OFFLINE"};

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

    rcll_vis_msgs::SetGameField gamefield_msg;
    gamefield_msg.walls = walls;
    gamefield_msg.insertion_zones = insertion_zones;
    gamefield_msg.field_length = 14.0;
    gamefield_msg.field_width = 8.0;
    gamefield_msg.zones_x = 14;
    gamefield_msg.zones_y = 8;
    pub_gamefield.publish(gamefield_msg);

    rcll_vis_msgs::Machines machines_msg;
    rcll_vis_msgs::Machine machine;
    std::vector<std::string> machines = {"BS", "DS", "SS", "CS1", "CS2", "RS1", "RS2"};
    std::vector<std::string> names = {"BaseStation", "DeliveryStation", "StorageStation", "CapStation 1", "CapStation 2", "RingStation 1", "RingStation 2"};
    std::vector<double> machine_pos_x = {2.5, 6.5, 6.5, 0.5, -4.5, -1.5, 3.5};
    std::vector<double> machine_pos_y = {2.5, 1.5, 5.5, 3.5, 4.5, 7.5, 6.5};
    std::vector<int> rot_c = {225, 135, 90, 270, 225, 180, 0};
    std::vector<int> rot_m = {315, 45, 270, 270, 315, 180, 180};
    std::vector<int> e1_status = {0, 1, 1, 1, 2, 2, 1};
    std::vector<int> e2_status = {0, 0, 1, 2, 0, 2, 1};

    for (size_t i = 0; i < machines.size(); i++){
        // cyan machine
        machine.name_short = machines[i];
        machine.name_long = names[i];
        machine.key = machines[i];
        machine.team = 0;
        machine.x = machine_pos_x[i];
        machine.y = machine_pos_y[i];
        machine.yaw = rot_c[i] / 180.0 * M_PI;
        int pstatus = i % machine_statuses.size();
        machine.machine_status_production = machine_statuses[pstatus];
        machine.machine_status_exploration1 = e1_status[i];
        machine.machine_status_exploration2 = e2_status[i];
        machines_msg.machines.push_back(machine);

        // magenta machine
        machine.name_short = machines[i];
        machine.name_long = names[i];
        machine.key = machines[i];
        machine.team = 1;
        machine.x = -machine_pos_x[i];
        machine.y = machine_pos_y[i];
        machine.yaw = rot_m[i] / 180.0 * M_PI;
        pstatus = i % machine_statuses.size();
        machine.machine_status_production = machine_statuses[pstatus];
        machine.machine_status_exploration1 = e1_status[machines.size() - i - 1];
        machine.machine_status_exploration2 = e2_status[machines.size() - i - 1];
        machines_msg.machines.push_back(machine);
    }
    pub_machines.publish(machines_msg);

    rcll_vis_msgs::Robots robots_msg;
    rcll_vis_msgs::Robot robot;
    std::vector<std::string> robot_keys = {"Rob1", "Rob2", "Rob3", "Rob1", "Rob2", "Rob3"};
    std::vector<std::string> robots = {"Ulrich", "Tim", "Christian", "Joerg", "Klaus", "Basti"};
    std::vector<int> robot_ids = {1, 2, 3, 1, 2, 3};
    std::vector<int> team = {0, 0, 0, 1, 1, 1};
    std::vector<double> robot_pos_x = {4.5, 5.5, 6.5, -4.5, -5.5, -6.5};
    std::vector<double> robot_pos_y = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    //std::vector<int> robot_rot = {90, 180, 180, 90, 0, 0};
    std::vector<int> robot_rot = {0, 0, 0, 45, 45, 45};
    std::vector<bool> robot_active = {true, true, true, true, true, false};
    std::vector<double> robot_active_time = {0.9, 0.8, 0.7, 1.0, 0.6, 0.0};
    std::vector<std::string> robot_status = {"Get cap at machine CS1 for product P1",
                                             "Get ring 2 at machine RS1 for product P7",
                                             "Deliver product P6 at DS",
                                             "Get cap at machine CS1 for product P1",
                                             "Get ring 2 at machine RS1 for product P7",
                                             ""};
    std::vector<int> robot_maintenance = {0, 0, 1, 0, 1, 0};

    rcll_vis_msgs::Products products_msg;
    std::vector<int> product_ids = {1, 5, 6, 7};
    std::vector<int> complexities = {0, 1, 2, 3};
    std::vector< std::vector<int> > structures = {{2, 0, 0, 0, 2}, {2, 1 , 0, 0, 2}, {1, 2, 3, 0, 1}, {1, 4, 4, 3, 1}};
    std::vector< std::vector<int> > step_stati_cyan = {{3, 0, 0, 0, 3, 3}, {3, 1, 0, 0, 0, 1}, {3, 3, 1, 0, 0, 1}, {3, 3, 3, 3, 3, 2}};
    std::vector< std::vector<int> > step_stati_magenta = {{3, 0, 0, 0, 1, 1}, {3, 3, 0, 0, 3, 3}, {3, 3, 3, 3, 3, 2}, {3, 3, 1, 0, 0, 1}};
    std::vector<double> progresss_cyan = {1.0, 0.33, 0.4, 0.95};
    std::vector<double> progresss_magenta = {0.33, 1.0, 0.95, 0.5};
    std::vector<int> points_cyan = {20, 0, 5, 25};
    std::vector<int> points_magenta = {0, 20, 50, 30};
    std::vector<int> points_max = {20, 20, 70, 45};
    std::vector<int> delivery_time = {567, 789, 123, 456};
    for (size_t i = 0; i < product_ids.size(); i++){
        rcll_vis_msgs::Product product;
        product.product_id = product_ids[i];
        product.quantity_id = 1;
        product.complexity = complexities[i];
        product.structure = structures[i];
        product.step_stati_cyan = step_stati_cyan[i];
        product.step_stati_magenta = step_stati_magenta[i];
        product.progress_cyan = progresss_cyan[i];
        product.progress_magenta = progresss_magenta[i];
        product.end_delivery_time = delivery_time[i];
        product.points_cyan = points_cyan[i];
        product.points_magenta = points_magenta[i];
        product.points_max = points_max[i];
        products_msg.orders.push_back(product);
    }

    rcll_vis_msgs::GameInfo gameinfo;
    gameinfo.team_name_cyan = "Carologistics";
    gameinfo.team_name_magenta = "GRIPS";
    gameinfo.team_points_cyan = 0;
    gameinfo.team_points_magenta = 0;
    gameinfo.game_state = 0;
    gameinfo.game_phase = 0;
    gameinfo.phase_time = 0.0;

    ROS_INFO("Entering loop");
    while(ros::ok()){
        // gameinfo
        gameinfo.game_state = 2;
        gameinfo.phase_time += loop_rate.expectedCycleTime().toSec();
        if (gameinfo.game_phase == 0 && gameinfo.phase_time > 2){ // set to SETUP
            gameinfo.phase_time = 0;
            gameinfo.game_state = 3;
            gameinfo.team_points_cyan = 0;
            gameinfo.team_points_magenta = 0;
            gameinfo.game_phase+=10;
            robot_pos_x = {4.5, 5.5, 6.5, -4.5, -5.5, -6.5};
            robot_pos_y = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
            //robot_rot = {90, 180, 180, 90, 0, 0};
        } else if (gameinfo.game_phase == 10 && gameinfo.phase_time > 2){ // set to EXPLORATION
            gameinfo.phase_time = 0;
            gameinfo.game_state = 2;
            gameinfo.team_points_cyan = 3;
            gameinfo.team_points_magenta = 3;
            gameinfo.game_phase+=10;
            robot_pos_x = {3.5, -5.0, -2.0, -1.5, 6.5, -6.5};
            robot_pos_y = {2.5, 5.0, 1.5, 5.5, 3.5, 2.5};
            //robot_rot = {10, 50, 100, 200, 150, 300};
        } else if (gameinfo.game_phase == 20 && gameinfo.phase_time > 2){ // set to PRODUCTION
            gameinfo.phase_time = 0;
            gameinfo.game_state = 2;
            gameinfo.team_points_cyan = 53;
            gameinfo.team_points_magenta = 103;
            gameinfo.game_phase+=10;
            robot_pos_x = {2.5, -4.0, -3.0, -4.5, 3.5, -1.5};
            robot_pos_y = {1.5, 6.0, 3.5, 3.5, 2.5, 5.5};
            //robot_rot = {50, 60, 220, 100, 300, 250};
        } else if (gameinfo.game_phase == 30 && gameinfo.phase_time > 10000){ // set to POST_GAME
            gameinfo.phase_time = 0;
            gameinfo.game_state = 3;
            gameinfo.game_phase+=10;
            gameinfo.team_points_cyan = 120;
            gameinfo.team_points_magenta = 120;
        }
        pub_gameinfo.publish(gameinfo);

        // robots
        robots_msg.robots.clear();
        for (size_t i = 0; i < robots.size(); i++){
            robot.key = robot_keys[i];
            robot.robot_name = robots[i];
            robot.robot_id = robot_ids[i];
            robot.team = team[i];
            robot.active = robot_active[i];
            robot.x = robot_pos_x[i];
            robot.y = robot_pos_y[i];
            robot.yaw = (robot_rot[i] + 90) / 180.0 * M_PI;
            robot.active_time = robot_active_time[i];
            robot.maintenance_count = robot_maintenance[i];
            robot.status = robot_status[i];
            robot.stamp.data = ros::Time::now();

            robots_msg.robots.push_back(robot);
        }
        pub_robots.publish(robots_msg);

        //machines status
        pub_machines.publish(machines_msg);
        pub_products.publish(products_msg);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
