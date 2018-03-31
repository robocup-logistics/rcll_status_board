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

    rcll_draw::FieldArea main_area_field;
}

void cb_gameinfo(const rcll_msgs::GameInfo::ConstPtr& msg){
    //ROS_INFO("Updating gameinfo");
    main_area_field.setTeam(msg->team_name_cyan, rcll_draw::CYAN);
    main_area_field.setTeam(msg->team_name_magenta, rcll_draw::MAGENTA);
    main_area_field.setGameInfo(gamestates[msg->game_state], gamephases[msg->game_phase], (int)msg->phase_time, msg->team_points_cyan, msg->team_points_magenta);
}

void cb_gamefield(const rcll_msgs::SetGameField::ConstPtr& msg){
    ROS_INFO("Initializing gamefield with w=%f h=%f zx=%i zy=%i", msg->field_w, msg->field_h, msg->zones_x, msg->zones_y);
    main_area_field.setLayout(msg->field_w, msg->field_h, msg->zones_x, msg->zones_y, msg->insertion_zones);
    main_area_field.setWalls(msg->walls);
}

void cb_set_machine(const rcll_msgs::SetMachines::ConstPtr& msg){
    ROS_INFO("Initializing machines");
    for (size_t i = 0; i < msg->machines.size(); i++){
        ROS_INFO("  name=%s team=%i index=%i", msg->machines[i].name_short.c_str(), msg->machines[i].team, msg->machines[i].index);
        main_area_field.setMachine(msg->machines[i].name_short, (rcll_draw::Team)msg->machines[i].team, msg->machines[i].index);
        main_area_field.setMachinePos(msg->machines[i].x, msg->machines[i].y, msg->machines[i].yaw, msg->machines[i].index);
    }
}

void cb_set_robot(const rcll_msgs::SetRobot::ConstPtr& msg){
    ROS_INFO("Initializing robot name=%s number=%i team=%i, index=%i", msg->robot_name.c_str(), msg->robot_id, msg->team, msg->index);
    main_area_field.addRobot(msg->robot_name, msg->robot_id, (rcll_draw::Team)msg->team);
}

void cb_robots(const rcll_msgs::Robots::ConstPtr& msg){
    for (size_t i = 0; i < msg->robots.size(); i++){
        main_area_field.setRobotPos(msg->robots[i].x, msg->robots[i].y, msg->robots[i].yaw, msg->robots[i].index);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "field_status_board");
    ros::NodeHandle nh;
    ros::Rate loop_rate(4.0);

    ros::Subscriber sub_gameinfo = nh.subscribe("refbox/gameinfo", 10, cb_gameinfo);
    ros::Subscriber sub_gamefield = nh.subscribe("refbox/set_gamefield", 10, cb_gamefield);
    ros::Subscriber sub_addrobot = nh.subscribe("refbox/set_robot", 10, cb_set_robot);
    ros::Subscriber sub_robots = nh.subscribe("refbox/update_robots", 10, cb_robots);
    ros::Subscriber sub_addmachine = nh.subscribe("refbox/set_machine", 10, cb_set_machine);


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

    //cv::setWindowProperty(title, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

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
