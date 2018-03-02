#ifndef RCLL_ROBOT_MARKER_H
#define RCLL_ROBOT_MARKER_H

#include <util.h>
#include <Arrow.h>
#include <Circle.h>
#include <Line.h>
#include <BoxLabel.h>

namespace rcll_draw {
    class RobotMarker {
    public:
        RobotMarker();
        RobotMarker(Team team);
        ~RobotMarker();

        void setOrigin(int x0, int y0, int pixel_per_meter);
        void setRobotParams(std::string name_str, int id, double d);
        void setPos(double x, double y, double yaw);
        void draw(cv::Mat &mat);

    private:
        int pixel_per_meter = 10;
        int x0, y0 = 0;

        Team team;
        std::string name;
        int id;
        double diameter;

        Circle crc_robot;
        Line ln_direction;
        BoxLabel blbl_id;
        Arrow arr_heading;
    };
}

#endif
