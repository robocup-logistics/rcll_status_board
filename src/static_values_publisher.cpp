#include <ros/ros.h>

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
    ros::init(argc, argv, "static_values_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2.0);

    ros::Publisher pub_gameinfo = nh.advertise<rcll_msgs::GameInfo>("refbox/gameinfo", 10);
    ros::Publisher pub_robots = nh.advertise<rcll_msgs::Robots>("refbox/update_robots", 10);
    ros::Publisher pub_setgamefield = nh.advertise<rcll_msgs::SetGameField>("refbox/set_gamefield", 10);
    ros::Publisher pub_setmachines = nh.advertise<rcll_msgs::SetMachines>("refbox/set_machine", 10);
    ros::Publisher pub_setrobot = nh.advertise<rcll_msgs::SetRobot>("refbox/set_robot", 10);
    ros::Publisher pub_machinesstatus = nh.advertise<rcll_msgs::MachinesStatus>("refbox/update_machines", 10);
    ros::Publisher pub_products = nh.advertise<rcll_msgs::Products>("refbox/update_products", 10);

    ros::Duration(1.0).sleep();

    std::vector<std::string> machine_statuses = {"Idle", "Broken", "Processing", "Processed", "Prepared", "Down", "Finished", "Waiting", "Offline"};


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

    rcll_msgs::SetMachines machines_init_msg;
    rcll_msgs::MachineInit machine_init;
    std::vector<std::string> machines = {"BS", "DS", "SS", "CS1", "CS2", "RS1", "RS2"};
    std::vector<std::string> names = {"BaseStation", "DeliveryStation", "StorageStation", "CapStation 1", "CapStation 2", "RingStation 1", "Ring Station 2"};
    std::vector<double> machine_pos_x = {2.5, 6.5, 6.5, 0.5, -4.5, -1.5, 3.5};
    std::vector<double> machine_pos_y = {2.5, 1.5, 5.5, 3.5, 4.5, 7.5, 6.5};
    std::vector<int> rot_c = {225, 135, 90, 270, 225, 180, 0};
    std::vector<int> rot_m = {315, 45, 270, 270, 315, 180, 180};

    for (size_t i = 0; i < machines.size(); i++){
        // cyan machine
        machine_init.name_short = machines[i];
        machine_init.name_long = names[i];
        machine_init.team = 0;
        machine_init.x = machine_pos_x[i];
        machine_init.y = machine_pos_y[i];
        machine_init.yaw = rot_c[i] / 180.0 * M_PI;
        machine_init.index = 2 * i;
        machines_init_msg.machines.push_back(machine_init);

        // magenta machine
        machine_init.name_short = machines[i];
        machine_init.name_long = names[i];
        machine_init.team = 1;
        machine_init.x = -machine_pos_x[i];
        machine_init.y = machine_pos_y[i];
        machine_init.yaw = rot_m[i] / 180.0 * M_PI;
        machine_init.index = 2 * i + 1;
        machines_init_msg.machines.push_back(machine_init);
    }
    pub_setmachines.publish(machines_init_msg);

    rcll_msgs::SetRobot robots_init_msg;
    std::vector<std::string> robots = {"Hans", "Peter", "Tim", "Joerg", "Klaus", "Bernd"};
    std::vector<int> robot_ids = {1, 2, 3, 1, 2, 3};
    std::vector<int> team = {0, 0, 0, 1, 1, 1};
    std::vector<double> robot_pos_x = {4.5, 5.5, 5.25, -5.5, -0.0, -6.25};
    std::vector<double> robot_pos_y = {3.5, 4.5, 1.5, 0.5, 2.5, 6.5};
    std::vector<int> robot_rot = {45, 125, 0, 305, 280, 180};
    for (size_t i = 0; i < robots.size(); i++){
        robots_init_msg.robot_name = robots[i];
        robots_init_msg.robot_id = robot_ids[i];
        robots_init_msg.team = team[i];
        robots_init_msg.index = i;

        pub_setrobot.publish(robots_init_msg);
    }

    rcll_msgs::Products products_msg;
    std::vector<int> product_ids = {1, 5, 6, 7};
    std::vector<int> complexities = {0, 1, 2, 3};
    std::vector< std::vector<int> > structures = {{2, 0, 0, 0, 2}, {2, 1 , 0, 0, 2}, {1, 2, 3, 0, 1}, {1, 4, 4, 3, 1}};
    std::vector< std::vector<int> > step_stati_cyan = {{3, 0, 0, 0, 3, 3}, {3, 1, 0, 0, 0, 1}, {3, 3, 1, 0, 0, 1}, {3, 3, 3, 3, 3, 2}};
    std::vector< std::vector<int> > step_stati_magenta = {{3, 0, 0, 0, 1, 1}, {3, 3, 0, 0, 0, 3}, {3, 3, 3, 3, 0, 2}, {3, 3, 1, 0, 0, 1}};
    std::vector<double> progresss_cyan = {1.0, 0.33, 0.4, 0.95};
    std::vector<double> progresss_magenta = {0.33, 1.0, 0.95, 0.5};
    std::vector<int> points_cyan = {20, 0, 5, 25};
    std::vector<int> points_magenta = {0, 20, 50, 30};
    std::vector<int> points_max = {20, 20, 70, 45};
    std::vector<int> delivery_time = {567, 789, 123, 456};
    for (size_t i = 0; i < product_ids.size(); i++){
        rcll_msgs::Product product;
        product.id = product_ids[i];
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

    rcll_msgs::GameInfo gameinfo;
    gameinfo.team_name_cyan = "Carologistics";
    gameinfo.team_name_magenta = "GRIPS";
    gameinfo.team_points_cyan = 10;
    gameinfo.team_points_magenta = 5;
    gameinfo.game_state = 0;
    gameinfo.game_phase = 0;
    gameinfo.phase_time = 0.0;

    rcll_msgs::Robots robots_update_msg;
    rcll_msgs::Robot robot_update;

    rcll_msgs::MachinesStatus machines_update_msg;
    rcll_msgs::MachineStatus machine_update;    

    ROS_INFO("Entering loop");
    int deg = 0;
    while(ros::ok()){
        // gameinfo
        gameinfo.game_state = 2;
        gameinfo.phase_time += loop_rate.expectedCycleTime().toSec();
        if (gameinfo.game_phase == 0 && gameinfo.phase_time > 10){ // set to SETUP
            gameinfo.phase_time = 0;
            gameinfo.game_state = 3;
            gameinfo.game_phase+=10;
        } else if (gameinfo.game_phase == 10 && gameinfo.phase_time > 10){ // set to EXPLORATION
            gameinfo.phase_time = 0;
            gameinfo.game_state = 2;
            gameinfo.game_phase+=10;
        } else if (gameinfo.game_phase == 20 && gameinfo.phase_time > 10){ // set to PRODUCTION
            gameinfo.phase_time = 0;
            gameinfo.game_state = 2;
            gameinfo.game_phase+=10;
        } else if (gameinfo.game_phase == 30 && gameinfo.phase_time > 10){ // set to POST_GAME
            gameinfo.phase_time = 0;
            gameinfo.game_state = 3;
            gameinfo.game_phase+=10;
        }
        pub_gameinfo.publish(gameinfo);

        // robots
        deg+=2;
        for(size_t i = 0; i < robots.size(); i++){
            robot_update.index = i;
            robot_update.x = robot_pos_x[i];
            robot_update.y = robot_pos_y[i];
            robot_update.yaw = (robot_rot[i] + deg) / 180.0 * M_PI;
            robot_update.active = true;
            robot_update.active_time = 0.0;
            robot_update.maintenance_count = 0;
            robot_update.status = "";

            robots_update_msg.robots.push_back(robot_update);
        }
        pub_robots.publish(robots_update_msg);

        //machines status
        for(size_t i = 0; i < machines.size(); i++){
            // cyan
            int pstatus = i % machine_statuses.size();
            int e1status = i % 3;
            int e2status = (2*i) % 3;
            machine_update.index = 2 * i;
            machine_update.machine_status_production = machine_statuses[pstatus];
            machine_update.machine_status_exploration1 = e1status;
            machine_update.machine_status_exploration2 = e2status;
            machines_update_msg.machines.push_back(machine_update);

            // magenta
            pstatus = i % machine_statuses.size();
            e1status = i % 3;
            e2status = (2*i) % 3;
            machine_update.index = 2 * i + 1;
            machine_update.machine_status_production = machine_statuses[pstatus];
            machine_update.machine_status_exploration1 = e1status;
            machine_update.machine_status_exploration2 = e2status;
            machines_update_msg.machines.push_back(machine_update);
        }
        pub_machinesstatus.publish(machines_update_msg);

        pub_products.publish(products_msg);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
