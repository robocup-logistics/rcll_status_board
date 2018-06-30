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

#include <constants.h>

#include <rcll_vis_msgs/GameInfo.h>
#include <rcll_vis_msgs/Products.h>
#include <rcll_vis_msgs/Machines.h>
#include <rcll_vis_msgs/Robots.h>
#include <rcll_vis_msgs/SetGameField.h>

/* =====================================================================================================|
nodename:           "static_values_publisher"

This node publishes static values and calls services once for testing purposes
====================================================================================================== */

using namespace rcll_draw;

struct Pose2D {
    float x;
    float y;
    float rot;
};

struct DummyRobot {
    std::string key;
    std::string name;
    int id;
    int team;
    Pose2D pose_setup;
    Pose2D pose_exploration;
    Pose2D pose_production;
    bool active;
    float active_time;
    std::string status;
    int maintenances;
};

struct DummyGameTeam {
    std::string team_name;
    int points_setup;
    int points_exploration;
    int points_production;
    int points_post_game;
};

struct DummyGame {
    DummyGameTeam team_cyan;
    DummyGameTeam team_magenta;
    int duration_pre_game;
    int duration_setup;
    int duration_exploration;
    int duration_production;
};

namespace {
    const std::vector<std::string> machine_statuses = {"IDLE", "BROKEN", "PROCESSING", "PROCESSED", "PREPARED", "DOWN", "READY-AT-OUTPUT", "WAIT-IDLE", "OFFLINE"};
}

void readGameField(ros::NodeHandle &nh, rcll_vis_msgs::SetGameField &gamefield_msg){
    nh.getParam("/rcll/gamefield/field_length", gamefield_msg.field_length);
    nh.getParam("/rcll/gamefield/field_width", gamefield_msg.field_width);
    nh.getParam("/rcll/gamefield/zones_x", gamefield_msg.zones_x);
    nh.getParam("/rcll/gamefield/zones_y", gamefield_msg.zones_y);
    nh.getParam("/rcll/gamefield/insertion_zones", gamefield_msg.insertion_zones);
    nh.getParam("/rcll/gamefield/walls", gamefield_msg.walls);
}

void readGame(ros::NodeHandle &nh, DummyGame &game, rcll_vis_msgs::GameInfo &gameinfo_msg){
    // cyan
    nh.getParam("/rcll/dummysetup/game/cyan/team_name", game.team_cyan.team_name);
    nh.getParam("/rcll/dummysetup/game/cyan/points_setup", game.team_cyan.points_setup);
    nh.getParam("/rcll/dummysetup/game/cyan/points_exploration", game.team_cyan.points_exploration);
    nh.getParam("/rcll/dummysetup/game/cyan/points_production", game.team_cyan.points_production);
    nh.getParam("/rcll/dummysetup/game/cyan/points_post_game", game.team_cyan.points_post_game);

    // magenta
    nh.getParam("/rcll/dummysetup/game/magenta/team_name", game.team_magenta.team_name);
    nh.getParam("/rcll/dummysetup/game/magenta/points_setup", game.team_magenta.points_setup);
    nh.getParam("/rcll/dummysetup/game/magenta/points_exploration", game.team_magenta.points_exploration);
    nh.getParam("/rcll/dummysetup/game/magenta/points_production", game.team_magenta.points_production);
    nh.getParam("/rcll/dummysetup/game/magenta/points_post_game", game.team_magenta.points_post_game);

    // phase durations
    nh.getParam("/rcll/dummysetup/game/duration/pre_game", game.duration_pre_game);
    nh.getParam("/rcll/dummysetup/game/duration/setup", game.duration_setup);
    nh.getParam("/rcll/dummysetup/game/duration/exploration", game.duration_exploration);
    nh.getParam("/rcll/dummysetup/game/duration/production", game.duration_production);

    gameinfo_msg.team_name_cyan = game.team_cyan.team_name;
    gameinfo_msg.team_name_magenta = game.team_magenta.team_name;
}

void readPose2D(ros::NodeHandle &nh, std::string key, float &pose_x, float &pose_y, float &pose_yaw){
    std::vector<float> coordinates;
    nh.getParam(key, coordinates);
    if (coordinates.size() == 3){
        pose_x = coordinates[0];
        pose_y = coordinates[1];
        pose_yaw = coordinates[2];
    }
}

void readMachineStatuses(ros::NodeHandle &nh, std::string key, rcll_vis_msgs::Machine &machine_msg){
    std::vector<int> statuses;
    nh.getParam(key, statuses);
    if (statuses.size() == 3){
        machine_msg.machine_status_exploration1 = statuses[0];
        machine_msg.machine_status_exploration2 = statuses[1];
        machine_msg.machine_status_production = machine_statuses[statuses[2] % machine_statuses.size()];
    }
}

void readMachines(ros::NodeHandle &nh, rcll_vis_msgs::Machines &machines_msg){
    std::vector<std::string> machine_keys;
    nh.getParam("/rcll/dummysetup/machines/names_short", machine_keys);

    for (size_t i = 0; i < machine_keys.size(); i++){
        // cyan and magenta
        rcll_vis_msgs::Machine machine_msg;
        machine_msg.key = machine_keys[i];
        machine_msg.name_short = machine_keys[i];
        nh.getParam("/rcll/dummysetup/machines/" + machine_keys[i] + "/name_long", machine_msg.name_long);

        // cyan
        machine_msg.team = CYAN;
        readPose2D(nh, "/rcll/dummysetup/machines/" + machine_keys[i] + "/cyan/pose", machine_msg.x, machine_msg.y, machine_msg.yaw);
        readMachineStatuses(nh, "/rcll/dummysetup/machines/" + machine_keys[i] + "/cyan/status", machine_msg);
        machines_msg.machines.push_back(machine_msg);

        // magenta
        machine_msg.team = MAGENTA;
        readPose2D(nh, "/rcll/dummysetup/machines/" + machine_keys[i] + "/magenta/pose", machine_msg.x, machine_msg.y, machine_msg.yaw);
        readMachineStatuses(nh, "/rcll/dummysetup/machines/" + machine_keys[i] + "/magenta/status", machine_msg);
        machines_msg.machines.push_back(machine_msg);
    }
}

void readProducts(ros::NodeHandle &nh, rcll_vis_msgs::Products &products_msg){
    std::vector<std::string> product_keys;
    nh.getParam("/rcll/dummysetup/products/list", product_keys);
    for (size_t i = 0; i < product_keys.size(); i++){
        rcll_vis_msgs::Product product_msg;
        // cyan and magenta
        nh.getParam("/rcll/dummysetup/products/" + product_keys[i] + "/order_id", product_msg.product_id);
        nh.getParam("/rcll/dummysetup/products/" + product_keys[i] + "/suborder_id", product_msg.quantity_id);
        nh.getParam("/rcll/dummysetup/products/" + product_keys[i] + "/complexity", product_msg.complexity);
        nh.getParam("/rcll/dummysetup/products/" + product_keys[i] + "/structure", product_msg.structure);
        nh.getParam("/rcll/dummysetup/products/" + product_keys[i] + "/points_max", product_msg.points_max);
        nh.getParam("/rcll/dummysetup/products/" + product_keys[i] + "/delivery_time", product_msg.end_delivery_time);

        // cyan
        nh.getParam("/rcll/dummysetup/products/" + product_keys[i] + "/cyan/step_stati", product_msg.step_stati_cyan);
        nh.getParam("/rcll/dummysetup/products/" + product_keys[i] + "/cyan/progress", product_msg.progress_cyan);
        nh.getParam("/rcll/dummysetup/products/" + product_keys[i] + "/cyan/points", product_msg.points_cyan);

        // magenta
        nh.getParam("/rcll/dummysetup/products/" + product_keys[i] + "/magenta/step_stati", product_msg.step_stati_magenta);
        nh.getParam("/rcll/dummysetup/products/" + product_keys[i] + "/magenta/progress", product_msg.progress_magenta);
        nh.getParam("/rcll/dummysetup/products/" + product_keys[i] + "/magenta/points", product_msg.points_magenta);
        products_msg.orders.push_back(product_msg);
    }
}

void readRobots(ros::NodeHandle &nh, std::vector<DummyRobot> &robots){
    std::vector<std::string> robot_keys;

    // cyan
    nh.getParam("/rcll/dummysetup/robots/cyan/keys", robot_keys);
    for (size_t i = 0; i < robot_keys.size(); i++){
        DummyRobot robot;
        robot.key = robot_keys[i];
        robot.team = CYAN;
        nh.getParam("/rcll/dummysetup/robots/cyan/" + robot_keys[i] + "/name", robot.name);
        nh.getParam("/rcll/dummysetup/robots/cyan/" + robot_keys[i] + "/id", robot.id);
        readPose2D(nh, "/rcll/dummysetup/robots/cyan/" + robot_keys[i] + "/pose_setup", robot.pose_setup.x, robot.pose_setup.y, robot.pose_setup.rot);
        readPose2D(nh, "/rcll/dummysetup/robots/cyan/" + robot_keys[i] + "/pose_exploration", robot.pose_exploration.x, robot.pose_exploration.y, robot.pose_exploration.rot);
        readPose2D(nh, "/rcll/dummysetup/robots/cyan/" + robot_keys[i] + "/pose_production", robot.pose_production.x, robot.pose_production.y, robot.pose_production.rot);
        nh.getParam("/rcll/dummysetup/robots/cyan/" + robot_keys[i] + "/active", robot.active);
        nh.getParam("/rcll/dummysetup/robots/cyan/" + robot_keys[i] + "/active_time", robot.active_time);
        nh.getParam("/rcll/dummysetup/robots/cyan/" + robot_keys[i] + "/status", robot.status);
        nh.getParam("/rcll/dummysetup/robots/cyan/" + robot_keys[i] + "/maintenances", robot.maintenances);
        robots.push_back(robot);
    }

    // magenta
    nh.getParam("/rcll/dummysetup/robots/magenta/keys", robot_keys);
    for (size_t i = 0; i < robot_keys.size(); i++){
        DummyRobot robot;
        robot.key = robot_keys[i];
        robot.team = MAGENTA;
        nh.getParam("/rcll/dummysetup/robots/magenta/" + robot_keys[i] + "/name", robot.name);
        nh.getParam("/rcll/dummysetup/robots/magenta/" + robot_keys[i] + "/id", robot.id);
        readPose2D(nh, "/rcll/dummysetup/robots/magenta/" + robot_keys[i] + "/pose_setup", robot.pose_setup.x, robot.pose_setup.y, robot.pose_setup.rot);
        readPose2D(nh, "/rcll/dummysetup/robots/magenta/" + robot_keys[i] + "/pose_exploration", robot.pose_exploration.x, robot.pose_exploration.y, robot.pose_exploration.rot);
        readPose2D(nh, "/rcll/dummysetup/robots/magenta/" + robot_keys[i] + "/pose_production", robot.pose_production.x, robot.pose_production.y, robot.pose_production.rot);
        nh.getParam("/rcll/dummysetup/robots/magenta/" + robot_keys[i] + "/active", robot.active);
        nh.getParam("/rcll/dummysetup/robots/magenta/" + robot_keys[i] + "/active_time", robot.active_time);
        nh.getParam("/rcll/dummysetup/robots/magenta/" + robot_keys[i] + "/status", robot.status);
        nh.getParam("/rcll/dummysetup/robots/magenta/" + robot_keys[i] + "/maintenances", robot.maintenances);
        robots.push_back(robot);
    }
}

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

    DummyGame game;
    std::vector<DummyRobot> robots;

    rcll_vis_msgs::SetGameField gamefield_msg;
    rcll_vis_msgs::GameInfo gameinfo_msg;
    rcll_vis_msgs::Machines machines_msg;
    rcll_vis_msgs::Products products_msg;
    rcll_vis_msgs::Robots robots_msg;
    rcll_vis_msgs::Robot robot_msg;

    readGame(nh, game, gameinfo_msg);
    readGameField(nh, gamefield_msg);
    readMachines(nh, machines_msg);
    readProducts(nh, products_msg);
    readRobots(nh, robots);

    pub_gamefield.publish(gamefield_msg);
    pub_machines.publish(machines_msg);

    ROS_INFO("Entering loop");
    while(ros::ok()){
        // gameinfo
        gameinfo_msg.game_state = PAUSED;
        gameinfo_msg.phase_time += loop_rate.expectedCycleTime().toSec();

        // check for change of gamephase
        if (gameinfo_msg.game_phase == PRE_GAME && gameinfo_msg.phase_time > game.duration_pre_game){ // set to SETUP
            gameinfo_msg.phase_time = 0;
            gameinfo_msg.game_state = RUNNING;
            gameinfo_msg.game_phase = SETUP;
            gameinfo_msg.team_points_cyan = game.team_cyan.points_setup;
            gameinfo_msg.team_points_magenta = game.team_magenta.points_setup;
        } else if (gameinfo_msg.game_phase == 10 && gameinfo_msg.phase_time > game.duration_setup){ // set to EXPLORATION
            gameinfo_msg.phase_time = 0;
            gameinfo_msg.game_state = RUNNING;
            gameinfo_msg.game_phase = EXPLORATION;
            gameinfo_msg.team_points_cyan = game.team_cyan.points_exploration;
            gameinfo_msg.team_points_magenta = game.team_magenta.points_exploration;
        } else if (gameinfo_msg.game_phase == 20 && gameinfo_msg.phase_time > game.duration_exploration){ // set to PRODUCTION
            gameinfo_msg.phase_time = 0;
            gameinfo_msg.game_state = RUNNING;
            gameinfo_msg.game_phase = PRODUCTION;
            gameinfo_msg.team_points_cyan = game.team_cyan.points_production;
            gameinfo_msg.team_points_magenta = game.team_magenta.points_production;
        } else if (gameinfo_msg.game_phase == 30 && gameinfo_msg.phase_time > game.duration_production){ // set to POST_GAME
            gameinfo_msg.phase_time = 0;
            gameinfo_msg.game_state = PAUSED;
            gameinfo_msg.game_phase = POST_GAME;
            gameinfo_msg.team_points_cyan = game.team_cyan.points_post_game;
            gameinfo_msg.team_points_magenta = game.team_magenta.points_post_game;
        }
        pub_gameinfo.publish(gameinfo_msg);

        // robots
        robots_msg.robots.clear();
        for (size_t i = 0; i < robots.size(); i++){
            robot_msg.key = robots[i].key;
            robot_msg.robot_name = robots[i].name;
            robot_msg.robot_id = robots[i].id;
            robot_msg.team = robots[i].team;
            robot_msg.active = robots[i].active;
            if (gameinfo_msg.game_phase == PRE_GAME || gameinfo_msg.game_phase == SETUP){
                robot_msg.x = robots[i].pose_setup.x;
                robot_msg.y = robots[i].pose_setup.y;
                robot_msg.yaw = robots[i].pose_setup.rot;
            } else if (gameinfo_msg.game_phase == EXPLORATION){
                robot_msg.x = robots[i].pose_exploration.x;
                robot_msg.y = robots[i].pose_exploration.y;
                robot_msg.yaw = robots[i].pose_exploration.rot;
            } else if (gameinfo_msg.game_phase == PRODUCTION || gameinfo_msg.game_phase == POST_GAME){
                robot_msg.x = robots[i].pose_production.x;
                robot_msg.y = robots[i].pose_production.y;
                robot_msg.yaw = robots[i].pose_production.rot;
            }

            robot_msg.active_time = robots[i].active_time;
            robot_msg.maintenance_count = robots[i].maintenances;
            robot_msg.status = robots[i].status;
            robot_msg.stamp.data = ros::Time::now();

            robots_msg.robots.push_back(robot_msg);
        }
        pub_robots.publish(robots_msg);

        // machines
        pub_machines.publish(machines_msg);

        // product
        pub_products.publish(products_msg);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
