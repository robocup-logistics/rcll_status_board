#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <rcll_status_board/GameInfo.h>
#include <rcll_status_board/Machines.h>
#include <rcll_status_board/Robots.h>
#include <rcll_status_board/Products.h>
#include <rcll_status_board/AddMachines.h>
#include <rcll_status_board/AddRobot.h>
#include <rcll_status_board/SetGameField.h>

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

void cb_gameinfo(const rcll_status_board::GameInfo::ConstPtr& msg){
    main_area_field.setTeam(msg->team_name_cyan, rcll_draw::CYAN);
    main_area_field.setTeam(msg->team_name_magenta, rcll_draw::MAGENTA);
    main_area_field.setGameInfo(gamestates[msg->game_state], gamephases[msg->game_phase], (int)msg->phase_time, msg->team_points_cyan, msg->team_points_magenta);
}

void cb_robots(const rcll_status_board::Robots::ConstPtr& msg){
    for (size_t i = 0; i < msg->robots.size(); i++){
        main_area_field.setRobotPos(msg->robots[i].x, msg->robots[i].y, msg->robots[i].yaw, msg->robots[i].index);
    }
}

bool cb_gamefield(rcll_status_board::SetGameField::Request &req, rcll_status_board::SetGameField::Response &res){
    ROS_INFO("Initializing gamefield with w=%f h=%f zx=%i zy=%i", req.field_w, req.field_h, req.zones_x, req.zones_y);
    main_area_field.setLayout(req.field_w, req.field_h, req.zones_x, req.zones_y, req.insertion_zones);
    main_area_field.setWalls(req.walls);
    return true;
}

bool cb_add_machine(rcll_status_board::AddMachines::Request &req, rcll_status_board::AddMachines::Response &res){
    ROS_INFO("Initializing machines");
    for (size_t i = 0; i < req.machines.size(); i++){
        main_area_field.addMachine(req.machines[i].name_short, (rcll_draw::Team)req.machines[i].team);
        main_area_field.setMachinePos(req.machines[i].x, req.machines[i].y, req.machines[i].yaw, i);
    }
    return true;
}

bool cb_add_robot(rcll_status_board::AddRobot::Request &req, rcll_status_board::AddRobot::Response &res){
    ROS_INFO("Initializing robot name=%s number=%i team=%i", req.robot_name.c_str(), req.robot_id, req.team);
    res.index = main_area_field.addRobot(req.robot_name, req.robot_id, (rcll_draw::Team)req.team);
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "field_status_board");
    ros::NodeHandle nh;
    ros::Rate loop_rate(4.0);

    ros::Subscriber sub_gameinfo = nh.subscribe("refbox/gameinfo", 10, cb_gameinfo);
    ros::Subscriber sub_robots = nh.subscribe("refbox/robots", 10, cb_robots);
    ros::ServiceServer srv_gamefield = nh.advertiseService("refbox/set_gamefield", cb_gamefield);
    ros::ServiceServer srv_addmachine = nh.advertiseService("refbox/add_machine", cb_add_machine);
    ros::ServiceServer srv_addrobot = nh.advertiseService("refbox/add_robot", cb_add_robot);

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
    rcll_draw::setImagePath("/home/faps/llerf2_ws/src/rcll_status_board/img/ppm/");

    int res_x = 1920;
    int res_y = 1080;
    int bordergapsize = 0.05 * res_y;
    int gapsize = 0.02 * res_y;

    cv::namedWindow(title, CV_WINDOW_NORMAL);

    cv::setWindowProperty(title, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    cv::Mat mat(res_y, res_x, CV_8UC4);
    rcll_draw::HeaderPanel header(title, team);
    header.setGeometry(bordergapsize, res_x, bordergapsize);
    main_area_field.setGeometry(bordergapsize, bordergapsize * 3, res_x - 2 * bordergapsize, res_y - 4 * bordergapsize, gapsize);

    while(ros::ok()){
        loop_rate.sleep();
        cv::rectangle(mat, cv::Point(0,0), cv::Point(res_x, res_y), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED, 0);
        header.draw(mat);
        main_area_field.draw(mat);
        cv::imshow(title, mat);
        cv::waitKey(3);
        ros::spinOnce();
    }
    return 0;
}
