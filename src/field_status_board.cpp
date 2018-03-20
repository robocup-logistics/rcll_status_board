#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <drawing.h>
#include <elements.h>

/* =====================================================================================================|
nodename:           "team_status_board"

This node draws the team status board for a given teamcolor
====================================================================================================== */

int main(int argc, char** argv){
    ros::init(argc, argv, "field_status_board");
    ros::NodeHandle nh;
    ros::Rate loop_rate(4.0);

    std::vector<double> walls = {
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

    ros::Time start = ros::Time::now();
    rcll_draw::Team team = rcll_draw::NO_TEAM;
    std::string title;
    rcll_draw::setImagePath("/home/faps/llerf2_ws/src/rcll_status_board/img/ppm/");

    title = "FIELD STATUS BOARD";

    int res_x = 1920;
    int res_y = 1080;
    int bordergapsize = 0.05 * res_y;
    int gapsize = 0.02 * res_y;

    cv::namedWindow(title, CV_WINDOW_NORMAL);

    //cv::setWindowProperty(title, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    cv::Mat mat(res_y, res_x, CV_8UC4);
    cv::rectangle(mat, cv::Point(0,0), cv::Point(res_x, res_y), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED, 0);

    rcll_draw::HeaderPanel header(title, team);
    header.setGeometry(bordergapsize, res_x, bordergapsize);
    header.draw(mat);

    double deg = 0;

    rcll_draw::FieldArea main_area_field;
    main_area_field.setGeometry(bordergapsize, bordergapsize * 3, res_x - 2 * bordergapsize, res_y - 4 * bordergapsize, gapsize);
    main_area_field.setTeam("Carologistics", rcll_draw::CYAN);
    main_area_field.setTeam("GRIPS", rcll_draw::MAGENTA);
    main_area_field.setLayout(14.0, 8.0, 14, 8, insertion_zones);
    main_area_field.setWalls(walls);

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
    main_area_field.setMachinePos(-3.5, 6.5, 180 * M_PI / 180.0, 13);

    std::string phase = "EXPLORATION";
    int gametime = 0;
    while(ros::ok()){
        gametime = (int)(ros::Time::now() - start).toSec();

        main_area_field.setGameInfo("RUNNING", phase, gametime, 50, 40);
        main_area_field.draw(mat);
        cv::imshow(title, mat);

        /*main_area_field.setRobotPos(4.5, 2.5, (45 + deg) * M_PI / 180.0, 0);
        main_area_field.setRobotPos(-5.5, 4.5, (321 + deg) * M_PI / 180.0, 1);*/

        deg+=1;
        cv::waitKey(3);
        loop_rate.sleep();
    }
    return 0;
}
