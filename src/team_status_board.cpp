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
    ros::init(argc, argv, "team_status_board");
    ros::NodeHandle nh;
    ros::Rate loop_rate(4.0);

    ros::Time start = ros::Time::now();
    rcll_draw::Team team = rcll_draw::MAGENTA;
    std::string title;
    rcll_draw::setImagePath("/home/faps/llerf2_ws/src/rcll_status_board/img/ppm/");

    if (team == rcll_draw::CYAN){
        title = "STATUS BOARD - CYAN";
    } else if (team == rcll_draw::MAGENTA){
        title = "STATUS BOARD - MAGENTA";
    } else {
        title = "STATUS BOARD";
    }

    // 0=not started, 1=construction, 2=delivery, 3=completed
    rcll_draw::Product p1;
    rcll_draw::Product p2;
    rcll_draw::Product p3;
    rcll_draw::Product p4;

    p1.complexity = 0;
    p1.base = 2;
    p1.ring1 = 0;
    p1.ring2 = 0;
    p1.ring3 = 0;
    p1.cap = 2;
    p1.status_base = 3;
    p1.status_ring1 = 0;
    p1.status_ring2 = 0;
    p1.status_ring3 = 0;
    p1.status_cap = 3;
    p1.status_product = 3;

    p2.complexity = 1;
    p2.base = 2;
    p2.ring1 = 1;
    p2.ring2 = 0;
    p2.ring3 = 0;
    p2.cap = 2;
    p2.status_base = 3;
    p2.status_ring1 = 1;
    p2.status_ring2 = 0;
    p2.status_ring3 = 0;
    p2.status_cap = 0;
    p2.status_product = 1;

    p3.complexity = 2;
    p3.base = 1;
    p3.ring1 = 2;
    p3.ring2 = 3;
    p3.ring3 = 0;
    p3.cap = 1;
    p3.status_base = 3;
    p3.status_ring1 = 3;
    p3.status_ring2 = 1;
    p3.status_ring3 = 0;
    p3.status_cap = 0;
    p3.status_product = 1;

    p4.complexity = 3;
    p4.base = 1;
    p4.ring1 = 4;
    p4.ring2 = 4;
    p4.ring3 = 3;
    p4.cap = 1;
    p4.status_base = 3;
    p4.status_ring1 = 3;
    p4.status_ring2 = 3;
    p4.status_ring3 = 3;
    p4.status_cap = 3;
    p4.status_product = 2;

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

    /*rcll_draw::TeamAreaProduction main_area_production(team);
    main_area_production.setGeometry(bordergapsize, bordergapsize * 3, res_x - 2 * bordergapsize, res_y - 4 * bordergapsize, gapsize);
    main_area_production.setGameInfo("RUNNING", "PRODUCTION", (int)(ros::Time::now() - start).toSec(), 50);
    main_area_production.setMachineName("Base Station (BS)", 0);
    main_area_production.setMachineName("Delivery Station (DS)", 1);
    main_area_production.setMachineName("Storage Station (SS)", 2);
    main_area_production.setMachineName("Cap Station 1 (CS1)", 3);
    main_area_production.setMachineName("Cap Station 2 (CS2)", 4);
    main_area_production.setMachineName("Ring Station 1 (RS1)", 5);
    main_area_production.setMachineName("Ring Station 2 (RS2)", 6);

    main_area_production.setMachineStatus("Idle", 0);
    main_area_production.setMachineStatus("Broken", 1);
    main_area_production.setMachineStatus("Processing", 2);
    main_area_production.setMachineStatus("Prepared", 3);
    main_area_production.setMachineStatus("Down", 4);
    main_area_production.setMachineStatus("Finished", 5);
    main_area_production.setMachineStatus("Waiting", 6);

    main_area_production.setRobotName("Robot 1", true, 0);
    main_area_production.setRobotName("Robot 2", true, 1);
    main_area_production.setRobotName("Robot 3", false, 2);

    main_area_production.setRobotStatus("Get ring 2 at machine RS1 for product P3", 0.98, 0, 1, 0);
    main_area_production.setRobotStatus("Get cap at machine CS1 for product P2", 0.55, 1, 1, 1);
    main_area_production.setRobotStatus("Offline", 0.0, 0, 0, 2);

    main_area_production.setProduct(1, p1, 1.0, 567, 20, 20, 0);
    main_area_production.setProduct(2, p2, 0.3, 789, 0, 20, 1);
    main_area_production.setProduct(3, p3, 0.4, 1345, 5, 70, 2);
    main_area_production.setProduct(4, p4, 0.95, 140, 25, 45, 3);*/

    rcll_draw::TeamAreaExploration main_area_exploration(team);
    main_area_exploration.setGeometry(bordergapsize, bordergapsize * 3, res_x - 2 * bordergapsize, res_y - 4 * bordergapsize, gapsize);
    main_area_exploration.setMachineName("Base Station (BS)", 0);
    main_area_exploration.setMachineName("Delivery Station (DS)", 1);
    main_area_exploration.setMachineName("Storage Station (SS)", 2);
    main_area_exploration.setMachineName("Cap Station 1 (CS1)", 3);
    main_area_exploration.setMachineName("Cap Station 2 (CS2)", 4);
    main_area_exploration.setMachineName("Ring Station 1 (RS1)", 5);
    main_area_exploration.setMachineName("Ring Station 2 (RS2)", 6);

    main_area_exploration.setMachineStatus(0, 1, 0);
    main_area_exploration.setMachineStatus(2, 0, 1);
    main_area_exploration.setMachineStatus(1, 2, 2);
    main_area_exploration.setMachineStatus(0, 1, 3);
    main_area_exploration.setMachineStatus(2, 2, 4);
    main_area_exploration.setMachineStatus(0, 0, 5);
    main_area_exploration.setMachineStatus(1, 1, 6);

    while(ros::ok()){
        //main_area_production.setGameInfo("RUNNING", "PRODUCTION", (int)(ros::Time::now() - start).toSec(), 50);
        main_area_exploration.setGameInfo("RUNNING", "EXPLORATION", (int)(ros::Time::now() - start).toSec(), 50, 40);
        //main_area_production.draw(mat);
        main_area_exploration.draw(mat);
        cv::imshow(title, mat);

        cv::waitKey(3);
        loop_rate.sleep();
    }
    return 0;
}
