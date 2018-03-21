#include <ros/ros.h>

#include <rcll_status_board/GameInfo.h>
#include <rcll_status_board/Machines.h>
#include <rcll_status_board/Robots.h>
#include <rcll_status_board/Products.h>
#include <rcll_status_board/AddMachines.h>
#include <rcll_status_board/SetGameField.h>


/* =====================================================================================================|
nodename:           "static_values_publisher"

This node publishes static values and calls services once for testing purposes
====================================================================================================== */

int main(int argc, char** argv){
    ros::init(argc, argv, "static_values_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2.0);


    ros::Publisher pub_gameinfo = nh.advertise<rcll_status_board::GameInfo>("refbox/gameinfo", 10);

    ros::ServiceClient sc_setgamefield = nh.serviceClient<rcll_status_board::SetGameField>("refbox/set_gamefield");
    ros::ServiceClient sc_addmachines = nh.serviceClient<rcll_status_board::AddMachines>("refbox/add_machine");

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

    rcll_status_board::SetGameField srv_gamefield;
    srv_gamefield.request.walls = walls;
    srv_gamefield.request.insertion_zones = insertion_zones;
    srv_gamefield.request.field_w = 14.0;
    srv_gamefield.request.field_h = 8.0;
    srv_gamefield.request.zones_x = 14;
    srv_gamefield.request.zones_y = 8;

    ros::service::waitForService("refbox/set_gamefield");
    if (sc_setgamefield.call(srv_gamefield)){
        ROS_INFO("Initialized gamefield");
    } else {
        ROS_WARN("Error when initializing gamefield");
    }

    rcll_status_board::AddMachines srv_machines;
    rcll_status_board::MachineInit machine;
    std::vector<std::string> machines = {"BS", "DS", "SS", "CS1", "CS2", "RS1", "RS2"};
    std::vector<std::string> names = {"BaseStation", "DeliveryStation", "StorageStation", "CapStation 1", "CapStation 2", "RingStation 1", "Ring Station 2"};
    std::vector<double> pos_x = {2.5, 6.5, 6.5, 0.5, -4.5, -1.5, 3.5};
    std::vector<double> pos_y = {2.5, 1.5, 5.5, 3.5, 4.5, 7.5, 6.5};
    std::vector<int> rot_c = {225, 135, 90, 270, 225, 180, 0};
    std::vector<int> rot_m = {315, 45, 270, 270, 315, 180, 180};

    for (size_t i = 0; i < machines.size(); i++){
        // cyan machine
        machine.name_short = machines[i];
        machine.name_long = names[i];
        machine.team = 0;
        machine.x = pos_x[i];
        machine.y = pos_y[i];
        machine.yaw = rot_c[i] / 180.0 * M_PI;
        srv_machines.request.machines.push_back(machine);

        // magenta machine
        machine.name_short = machines[i];
        machine.name_long = names[i];
        machine.team = 1;
        machine.x = -pos_x[i];
        machine.y = pos_y[i];
        machine.yaw = rot_m[i] / 180.0 * M_PI;
        srv_machines.request.machines.push_back(machine);
    }

    ros::service::waitForService("refbox/add_machine");
    if (sc_addmachines.call(srv_machines)){
        ROS_INFO("Initialized machines");
    } else {
        ROS_WARN("Error when initializing machines");
    }

    rcll_status_board::GameInfo gameinfo;
    gameinfo.team_name_cyan = "Carologistics";
    gameinfo.team_name_magenta = "GRIPS";
    gameinfo.team_points_cyan = 10;
    gameinfo.team_points_magenta = 5;
    gameinfo.game_state = 0;
    gameinfo.game_phase = 0;
    gameinfo.phase_time = 0.0;
    /*
    main_area_field.addRobot("1", 1, rcll_draw::CYAN);
    main_area_field.setRobotPos(4.5, 2.5, 45 * M_PI / 180.0, 0);
    main_area_field.addRobot("1", 1, rcll_draw::MAGENTA);
    main_area_field.setRobotPos(-5.5, 4.5, 321 * M_PI / 180.0, 1);

    main_area_field.addMachine("BS", rcll_draw::CYAN);
    main_area_field.setMachinePos(2.5, 2.5, 225 * M_PI / 180.0, 0);
    main_area_field.addMachine("BS", rcll_draw::MAGENTA);
    main_area_field.setMachinePos(-2.5, 2.5, 315 * M_PI / 180.0, 1);
    main_area_field.addMachine("DS", rcll_draw::CYAN);
    main_area_field.setMachinePos(6.5, 1.5, 135 * M_PI / 180.0, 2);
    main_area_field.addMachine("DS", rcll_draw::MAGENTA);
    main_area_field.setMachinePos(-6.5, 1.5, 45 * M_PI / 180.0, 3);
    main_area_field.addMachine("SS", rcll_draw::CYAN);
    main_area_field.setMachinePos(6.5, 5.5, 90 * M_PI / 180.0, 4);
    main_area_field.addMachine("SS", rcll_draw::MAGENTA);
    main_area_field.setMachinePos(-6.5, 5.5, 270 * M_PI / 180.0, 5);
    main_area_field.addMachine("CS1", rcll_draw::CYAN);
    main_area_field.setMachinePos(0.5, 3.5, 270 * M_PI / 180.0, 6);
    main_area_field.addMachine("CS1", rcll_draw::MAGENTA);
    main_area_field.setMachinePos(-0.5, 3.5, 270 * M_PI / 180.0, 7);
    main_area_field.addMachine("CS2", rcll_draw::CYAN);
    main_area_field.setMachinePos(-4.5, 4.5, 225 * M_PI / 180.0, 8);
    main_area_field.addMachine("CS2", rcll_draw::MAGENTA);
    main_area_field.setMachinePos(4.5, 4.5, 315 * M_PI / 180.0, 9);
    main_area_field.addMachine("RS1", rcll_draw::CYAN);
    main_area_field.setMachinePos(-1.5, 7.5, 180 * M_PI / 180.0, 10);
    main_area_field.addMachine("RS1", rcll_draw::MAGENTA);
    main_area_field.setMachinePos(1.5, 7.5, 180 * M_PI / 180.0, 11);
    main_area_field.addMachine("RS2", rcll_draw::CYAN);
    main_area_field.setMachinePos(3.5, 6.5, 0 * M_PI / 180.0, 12);
    main_area_field.addMachine("RS2", rcll_draw::MAGENTA);
    main_area_field.setMachinePos(-3.5, 6.5, 180 * M_PI / 180.0, 13);*/

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

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
