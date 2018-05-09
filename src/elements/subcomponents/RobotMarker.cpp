/*
The MIT License (MIT)

Copyright (c) 2017-2018 Florian Eith <florian.eith@web.de>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <RobotMarker.h>

// RobotMarker ####################################################################
rcll_draw::RobotMarker::RobotMarker(Team team){
    blbl_id.setAlignment(rcll_draw::CenterCenter);

    if (team == rcll_draw::CYAN){
        crc_robot.setBackgroundColor(rcll_draw::C_CYAN_LIGHT);
    } else if(team == rcll_draw::MAGENTA){
        crc_robot.setBackgroundColor(rcll_draw::C_MAGENTA_LIGHT);
    } else {
        crc_robot.setBackgroundColor(rcll_draw::C_WHITE);
    }
    blbl_id.setBackgroundColor(rcll_draw::C_TRANSPARENT);

    crc_robot.setBorderColor(rcll_draw::C_BLACK);
    ln_direction.setBorderColor(rcll_draw::C_BLACK);
    blbl_id.setBorderColor(rcll_draw::C_TRANSPARENT);
    arr_heading.setColor(rcll_draw::C_BLACK);

    crc_robot.setBorderSize(2);
    ln_direction.setBorderSize(2);

    blbl_id.setFontSize(0.6);
    blbl_id.setFrontColor(rcll_draw::C_BLACK);
}

rcll_draw::RobotMarker::~RobotMarker(){

}

void rcll_draw::RobotMarker::setOrigin(int origin_x, int origin_y, int pixel_per_meter){
    this->origin_x = origin_x;
    this->origin_y = origin_y;
    this->pixel_per_meter = pixel_per_meter;
}

void rcll_draw::RobotMarker::setRefBoxView(bool refbox_view){
    this->refbox_view = refbox_view;
}

void rcll_draw::RobotMarker::setRobot(rcll_vis_msgs::Robot &robot){
    double ang;
    int offset_x, offset_y, r1_x, r1_y, dst;

    this->robot = robot;
    this->diameter = 0.5;

    if (refbox_view){
        ang = -(robot.yaw);
        offset_x = origin_x + robot.x * pixel_per_meter;
        offset_y = origin_y - robot.y * pixel_per_meter;
    } else {
        ang = -(robot.yaw+M_PI);
        offset_x = origin_x - robot.x * pixel_per_meter;
        offset_y = origin_y + robot.y * pixel_per_meter;
    }

    r1_x = diameter/4 * cos(ang - M_PI/2) * pixel_per_meter;
    r1_y = diameter/4 * sin(ang - M_PI/2) * pixel_per_meter;
    dst = pixel_per_meter * diameter / 2;

    blbl_id.setContent(std::to_string(robot.robot_id));

    crc_robot.setPos(offset_x, offset_y);
    crc_robot.setSize(dst);
    arr_heading.setArrowByLength(offset_x, offset_y, (ang), dst);
    blbl_id.setPos(offset_x + r1_x - dst, offset_y + r1_y - dst);
    blbl_id.setSize(dst * 2, dst * 2);

    pose_set = (sqrt(robot.x * robot.x + robot.y * robot.y) > 0.01);
}

void rcll_draw::RobotMarker::draw(cv::Mat &mat){
    if (robot.active && pose_set && ((ros::Time::now() - robot.stamp.data).toSec() < 5.0)){
        crc_robot.draw(mat);
        blbl_id.draw(mat);
        arr_heading.draw(mat);
    }
}
