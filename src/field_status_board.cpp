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

    rcll_draw::FieldArea main_area_field;
    main_area_field.setGeometry(bordergapsize, bordergapsize * 3, res_x - 2 * bordergapsize, res_y - 4 * bordergapsize, gapsize);
    main_area_field.setTeam("Carologistics", rcll_draw::CYAN);
    main_area_field.setTeam("GRIPS", rcll_draw::MAGENTA);
    main_area_field.setLayout(14.0, 8.0, 14, 8, insertion_zones);
    main_area_field.setWalls(walls);

    std::string phase = "PRE GAME";
    int gametime = 0;
    while(ros::ok()){
        gametime = (int)(ros::Time::now() - start).toSec();

        main_area_field.setGameInfo("RUNNING", phase, gametime, 50, 40);
        main_area_field.draw(mat);
        cv::imshow(title, mat);

        cv::waitKey(3);
        loop_rate.sleep();
    }
    return 0;
}
