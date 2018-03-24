#include <ros/ros.h>

#include <rcll_msgs/GameInfo.h>
#include <rcll_msgs/MachinesStatus.h>
#include <rcll_msgs/Robots.h>
#include <rcll_msgs/Products.h>
#include <rcll_msgs/AddMachines.h>
#include <rcll_msgs/AddRobot.h>
#include <rcll_msgs/SetGameField.h>


/* =====================================================================================================|
nodename:           "static_values_publisher"

This node publishes static values and calls services once for testing purposes
====================================================================================================== */

int main(int argc, char** argv){
    ros::init(argc, argv, "static_values_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2.0);


    ros::Publisher pub_gameinfo = nh.advertise<rcll_msgs::GameInfo>("refbox/gameinfo", 10);
    ros::Publisher pub_robots = nh.advertise<rcll_msgs::Robots>("refbox/robots", 10);

    ros::ServiceClient sc_setgamefield = nh.serviceClient<rcll_msgs::SetGameField>("refbox/set_gamefield");
    ros::ServiceClient sc_addmachines = nh.serviceClient<rcll_msgs::AddMachines>("refbox/add_machine");
    ros::ServiceClient sc_addrobot = nh.serviceClient<rcll_msgs::AddRobot>("refbox/add_robot");

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

    rcll_msgs::SetGameField srv_gamefield;
    srv_gamefield.request.walls = walls;
    srv_gamefield.request.insertion_zones = insertion_zones;
    srv_gamefield.request.field_w = 14.0;
    srv_gamefield.request.field_h = 8.0;
    srv_gamefield.request.zones_x = 14;
    srv_gamefield.request.zones_y = 8;

    ros::service::waitForService("refbox/set_gamefield");
    if (sc_setgamefield.call(srv_gamefield)){
        ROS_DEBUG("Initialized gamefield");
    } else {
        ROS_WARN("Error when initializing gamefield");
    }

    rcll_msgs::AddMachines srv_machines;
    rcll_msgs::MachineInit machine;
    std::vector<std::string> machines = {"BS", "DS", "SS", "CS1", "CS2", "RS1", "RS2"};
    std::vector<std::string> names = {"BaseStation", "DeliveryStation", "StorageStation", "CapStation 1", "CapStation 2", "RingStation 1", "Ring Station 2"};
    std::vector<double> machine_pos_x = {2.5, 6.5, 6.5, 0.5, -4.5, -1.5, 3.5};
    std::vector<double> machine_pos_y = {2.5, 1.5, 5.5, 3.5, 4.5, 7.5, 6.5};
    std::vector<int> rot_c = {225, 135, 90, 270, 225, 180, 0};
    std::vector<int> rot_m = {315, 45, 270, 270, 315, 180, 180};

    for (size_t i = 0; i < machines.size(); i++){
        // cyan machine
        machine.name_short = machines[i];
        machine.name_long = names[i];
        machine.team = 0;
        machine.x = machine_pos_x[i];
        machine.y = machine_pos_y[i];
        machine.yaw = rot_c[i] / 180.0 * M_PI;
        srv_machines.request.machines.push_back(machine);

        // magenta machine
        machine.name_short = machines[i];
        machine.name_long = names[i];
        machine.team = 1;
        machine.x = -machine_pos_x[i];
        machine.y = machine_pos_y[i];
        machine.yaw = rot_m[i] / 180.0 * M_PI;
        srv_machines.request.machines.push_back(machine);
    }

    ros::service::waitForService("refbox/add_machine");
    if (sc_addmachines.call(srv_machines)){
        ROS_DEBUG("Initialized machines");
    } else {
        ROS_WARN("Error when initializing machines");
    }

    rcll_msgs::AddRobot srv_robots;
    std::vector<std::string> robots = {"R1", "R2", "R3", "R1", "R2", "R3"};
    std::vector<int> robot_ids = {1, 2, 3, 1, 2, 3};
    std::vector<int> team = {0, 0, 0, 1, 1, 1};
    std::vector<double> robot_pos_x = {4.5, 5.5, 5.25, -5.5, -0.0, -6.25};
    std::vector<double> robot_pos_y = {3.5, 4.5, 1.5, 0.5, 2.5, 6.5};
    std::vector<int> robot_rot = {45, 125, 0, 305, 280, 180};
    std::vector<int> robot_index;
    for (size_t i = 0; i < robots.size(); i++){
        srv_robots.request.robot_name = robots[i];
        srv_robots.request.robot_id = robot_ids[i];
        srv_robots.request.team = team[i];

        ros::service::waitForService("refbox/add_robot");
        if (sc_addrobot.call(srv_robots)){
            ROS_DEBUG("Initialized robot %lu", i);
            robot_index.push_back(srv_robots.response.index);
        } else {
            ROS_WARN("Error when initializing robots");
        }
    }

    rcll_msgs::GameInfo gameinfo;
    gameinfo.team_name_cyan = "Carologistics";
    gameinfo.team_name_magenta = "GRIPS";
    gameinfo.team_points_cyan = 10;
    gameinfo.team_points_magenta = 5;
    gameinfo.game_state = 0;
    gameinfo.game_phase = 0;
    gameinfo.phase_time = 0.0;

    rcll_msgs::Robots robots_update;
    rcll_msgs::Robot robot;
    for(size_t i = 0; i < robot_index.size(); i++){
        robot.index = robot_index[i];
        robot.x = robot_pos_x[i];
        robot.y = robot_pos_y[i];
        robot.yaw = robot_rot[i] / 180.0 * M_PI;
        robots_update.robots.push_back(robot);
    }
    pub_robots.publish(robots_update);

    ROS_INFO("Entering loop");
    int deg = 0;
    while(ros::ok()){
        // gameinfo
        gameinfo.game_state = 2;
        gameinfo.phase_time += loop_rate.expectedCycleTime().toSec();
        if (gameinfo.game_phase == 0 && gameinfo.phase_time > 30){ // set to SETUP
            gameinfo.phase_time = 0;
            gameinfo.game_state = 3;
            gameinfo.game_phase+=10;
        } else if (gameinfo.game_phase == 10 && gameinfo.phase_time > 300){ // set to EXPLORATION
            gameinfo.phase_time = 0;
            gameinfo.game_state = 2;
            gameinfo.game_phase+=10;
        } else if (gameinfo.game_phase == 20 && gameinfo.phase_time > 180){ // set to PRODUCTION
            gameinfo.phase_time = 0;
            gameinfo.game_state = 2;
            gameinfo.game_phase+=10;
        } else if (gameinfo.game_phase == 30 && gameinfo.phase_time > 1200){ // set to POST_GAME
            gameinfo.phase_time = 0;
            gameinfo.game_state = 3;
            gameinfo.game_phase+=10;
        }
        pub_gameinfo.publish(gameinfo);

        // robots
        deg+=2;
        for(size_t i = 0; i < robot_index.size(); i++){
            robot.index = robot_index[i];
            robot.x = robot_pos_x[i];
            robot.y = robot_pos_y[i];
            robot.yaw = (robot_rot[i] + deg) / 180.0 * M_PI;
            robots_update.robots.push_back(robot);
        }
        pub_robots.publish(robots_update);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
