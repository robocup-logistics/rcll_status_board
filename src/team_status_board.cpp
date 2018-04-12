#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <rcll_msgs/GameInfo.h>
#include <rcll_msgs/MachinesStatus.h>
#include <rcll_msgs/Robots.h>
#include <rcll_msgs/Products.h>
#include <rcll_msgs/SetMachines.h>
#include <rcll_msgs/SetRobot.h>
#include <rcll_msgs/SetGameField.h>

#include <drawing.h>
#include <elements.h>

/* =====================================================================================================|
nodename:           "team_status_board"

This node draws the team status board for a given teamcolor
====================================================================================================== */

namespace {
    std::map<int, std::string> gamestates;
    std::map<int, std::string> gamephases;

    rcll_draw::GamePhase gamephase;

    int robots = 0;
    std::map<int, int> machine_indices; // maps from global index (both teams) to local index (team for this view)
    std::map<int, int> robot_indices; // maps from global index (both teams) to local index (team for this view)

    rcll_draw::Team team = rcll_draw::NO_TEAM;

    rcll_draw::TeamAreaPreGameSetup main_area_pregamesetup;
    rcll_draw::TeamAreaExploration main_area_exploration;
    rcll_draw::TeamAreaProduction main_area_production;
    rcll_draw::TeamAreaPostGame main_area_postgame;
}

void cb_gameinfo(const rcll_msgs::GameInfo::ConstPtr& msg){
    main_area_pregamesetup.setTeams(msg->team_name_cyan, msg->team_name_magenta);
    main_area_pregamesetup.setGameInfo(gamestates[msg->game_state], gamephases[msg->game_phase], (int)msg->phase_time, msg->team_points_cyan, msg->team_points_magenta);
    main_area_exploration.setGameInfo(gamestates[msg->game_state], gamephases[msg->game_phase], (int)msg->phase_time, msg->team_points_cyan, msg->team_points_magenta);
    if (team == rcll_draw::CYAN){
        main_area_production.setGameInfo(gamestates[msg->game_state], gamephases[msg->game_phase], (int)msg->phase_time, msg->team_points_cyan);
    } else if (team == rcll_draw::MAGENTA){
        main_area_production.setGameInfo(gamestates[msg->game_state], gamephases[msg->game_phase], (int)msg->phase_time, msg->team_points_magenta);
    } else {
        main_area_production.setGameInfo(gamestates[msg->game_state], gamephases[msg->game_phase], (int)msg->phase_time, 0);
    }
    main_area_postgame.setTeams(msg->team_name_cyan, msg->team_name_magenta);
    main_area_postgame.setGameInfo(gamestates[msg->game_state], gamephases[msg->game_phase], (int)msg->phase_time, msg->team_points_cyan, msg->team_points_magenta);

    gamephase = (rcll_draw::GamePhase)msg->game_phase;
}

void cb_set_machine(const rcll_msgs::SetMachines::ConstPtr& msg){
    ROS_INFO("Initializing machines");
    int c = 0;
    for (size_t i = 0; i < msg->machines.size(); i++){
        if ((rcll_draw::Team)msg->machines[i].team == team){
            ROS_INFO("  name=%s team=%i global_index=%i local_index=%i", msg->machines[i].name_short.c_str(), msg->machines[i].team, msg->machines[i].index, c);
            main_area_production.setMachineName(msg->machines[i].name_long, msg->machines[i].name_short, c);
            main_area_exploration.setMachineName(msg->machines[i].name_long, msg->machines[i].name_short, c);
            machine_indices[msg->machines[i].index] = c++;
        } else {
            machine_indices[msg->machines[i].index] = -1;
        }
    }
}

void cb_machines(const rcll_msgs::MachinesStatus::ConstPtr& msg){
    for (size_t i = 0; i < msg->machines.size(); i++){
        if (machine_indices[msg->machines[i].index] >= 0){
            main_area_production.setMachineStatus(msg->machines[i].machine_status_production, machine_indices[msg->machines[i].index]);
            main_area_exploration.setMachineStatus(msg->machines[i].machine_status_exploration1, msg->machines[i].machine_status_exploration2, machine_indices[msg->machines[i].index]);
        }
    }
}

void cb_set_robot(const rcll_msgs::SetRobot::ConstPtr& msg){
    if ((rcll_draw::Team)msg->team == team){
        robot_indices[msg->index] = robots++;
        ROS_INFO("Initializing robot name=%s number=%i team=%i, globalindex=%i, localindex=%i", msg->robot_name.c_str(), msg->robot_id, msg->team, msg->index, robot_indices[msg->index]);
        main_area_production.setRobotName(msg->robot_id, msg->robot_name, msg->active, robot_indices[msg->index]);
    } else {
        robot_indices[msg->index] = robot_indices.size()-1;
    }
}

void cb_robots(const rcll_msgs::Robots::ConstPtr& msg){
    for (size_t i = 0; i < msg->robots.size(); i++){
        if (robot_indices[msg->robots[i].index] >= 0){
            main_area_production.setRobotStatus(msg->robots[i].status, msg->robots[i].active_time, msg->robots[i].maintenance_count, 1, robot_indices[msg->robots[i].index]);
        }
    }
}

void cb_products(const rcll_msgs::Products::ConstPtr& msg){
    main_area_production.setProductsCount(msg->orders.size());
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

        main_area_production.setProduct(pi, i);
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
    ros::Subscriber sub_robots = nh.subscribe("refbox/update_robots", 10, cb_robots);
    ros::Subscriber sub_setmachine = nh.subscribe("refbox/set_machines", 10, cb_set_machine);
    ros::Subscriber sub_setrobot = nh.subscribe("refbox/set_robot", 10, cb_set_robot);
    ros::Subscriber sub_machines = nh.subscribe("refbox/update_machines", 10, cb_machines);
    ros::Subscriber sub_products = nh.subscribe("refbox/update_products", 10, cb_products);

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

    gamestates[0] = "INIT";
    gamestates[1] = "WAIT START";
    gamestates[2] = "RUNNING";
    gamestates[3] = "PAUSED";

    gamephases[0] = "PRE GAME";
    gamephases[10] = "SETUP";
    gamephases[20] = "EXPLORATION";
    gamephases[30] = "PRODUCTION";
    gamephases[40] = "POST GAME";

    int bordergapsize = 0.05 * res_y;
    int gapsize = 0.02 * res_y;

    cv::namedWindow(title, CV_WINDOW_NORMAL);

    if (fullscreen){
        cv::setWindowProperty(title, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    }

    cv::Mat mat(res_y, res_x, CV_8UC4);
    cv::rectangle(mat, cv::Point(0,0), cv::Point(res_x, res_y), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED, 0);

    rcll_draw::HeaderPanel header(title, team);
    header.setGeometry(bordergapsize, res_x, bordergapsize);
    header.draw(mat);

    main_area_pregamesetup = rcll_draw::TeamAreaPreGameSetup();
    main_area_pregamesetup.setGeometry(bordergapsize, bordergapsize * 3, res_x - 2 * bordergapsize, res_y - 4 * bordergapsize, gapsize);

    main_area_exploration = rcll_draw::TeamAreaExploration(team);
    main_area_exploration.setGeometry(bordergapsize, bordergapsize * 3, res_x - 2 * bordergapsize, res_y - 4 * bordergapsize, gapsize);

    main_area_production = rcll_draw::TeamAreaProduction(team);
    main_area_production.setGeometry(bordergapsize, bordergapsize * 3, res_x - 2 * bordergapsize, res_y - 4 * bordergapsize, gapsize);

    main_area_postgame = rcll_draw::TeamAreaPostGame();
    main_area_postgame.setGeometry(bordergapsize, bordergapsize * 3, res_x - 2 * bordergapsize, res_y - 4 * bordergapsize, gapsize);

    double paging_timer = 0.0;
    while(ros::ok() && cvGetWindowHandle(title.c_str())){
        loop_rate.sleep();
        cv::rectangle(mat, cv::Point(0,0), cv::Point(res_x, res_y), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED, 0);
        header.draw(mat);

        if(gamephase == rcll_draw::PRE_GAME){
            main_area_pregamesetup.draw(mat);
        } else if(gamephase == rcll_draw::SETUP){
            main_area_pregamesetup.draw(mat);
        } else if (gamephase == rcll_draw::EXPLORATION){
            main_area_exploration.draw(mat);
        } else if (gamephase == rcll_draw::PRODUCTION){

            paging_timer +=loop_rate.expectedCycleTime().toSec();

            if (paging_timer > product_page_time){
                main_area_production.paging();
                paging_timer = 0.0;
            }

            main_area_production.draw(mat);
        } else if (gamephase == rcll_draw::POST_GAME){
            main_area_postgame.draw(mat);
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
