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

void rcll_draw::RobotMarker::setOrigin(int x0, int y0, int pixel_per_meter){
    this->x0 = x0;
    this->y0 = y0;
    this->pixel_per_meter = pixel_per_meter;
}

void rcll_draw::RobotMarker::setPos(double x, double y, double yaw){
    double ang = -(yaw+M_PI);
    int offset_x = x0 - x * pixel_per_meter;
    int offset_y = y0 + y * pixel_per_meter;
    int r1_x = diameter/4 * cos(ang-M_PI/2) * pixel_per_meter;
    int r1_y = diameter/4 * sin(ang-M_PI/2) * pixel_per_meter;
    int dst = pixel_per_meter * diameter / 2;
    crc_robot.setPos(offset_x, offset_y);
    crc_robot.setSize(dst);
    arr_heading.setArrowByLength(offset_x, offset_y, ang, dst);
    blbl_id.setPos(offset_x + r1_x - dst, offset_y + r1_y - dst);
    blbl_id.setSize(dst * 2, dst * 2);
    pose_set = true;
}

void rcll_draw::RobotMarker::setRobotParams(std::string name_str, int id, double d){
    this->name = name_str;
    this->id = id;
    this->diameter = d;
    blbl_id.setContent(std::to_string(id));
}

void rcll_draw::RobotMarker::draw(cv::Mat &mat){
    if (pose_set){
        crc_robot.draw(mat);
        blbl_id.draw(mat);
        arr_heading.draw(mat);
    }
}
