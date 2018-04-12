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

#include <RobotLabel.h>

// RobotLabel ####################################################################
rcll_draw::RobotLabel::RobotLabel(){
    blbl_name.setAlignment(rcll_draw::CenterLeft);
    mlblbl_activity.setAlignment(rcll_draw::TopLeft);
    blbl_activetime.setAlignment(rcll_draw::CenterLeft);
    blbl_maintenance.setAlignment(rcll_draw::CenterLeft);

    blbl_name.setBackgroundColor(rcll_draw::C_BLACK);
    mlblbl_activity.setBackgroundColor(rcll_draw::C_GREY_DARK);
    blbl_activetime.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_maintenance.setBackgroundColor(rcll_draw::C_GREY_DARK);

    blbl_name.setBorderColor(rcll_draw::C_WHITE);
    mlblbl_activity.setBorderColor(rcll_draw::C_WHITE);
    blbl_activetime.setBorderColor(rcll_draw::C_WHITE);
    blbl_maintenance.setBorderColor(rcll_draw::C_WHITE);

    blbl_name.setFontSize(1.0);
    mlblbl_activity.setFontSize(0.7);
    blbl_activetime.setFontSize(0.7);
    blbl_maintenance.setFontSize(0.7);

    blbl_name.setFrontColor(rcll_draw::C_WHITE);
    mlblbl_activity.setFrontColor(rcll_draw::C_BLACK);
    blbl_activetime.setFrontColor(rcll_draw::C_BLACK);
    blbl_maintenance.setFrontColor(rcll_draw::C_BLACK);

    mlblbl_activity.setContent("Activity: unknown");
    blbl_activetime.setContent(" Active Time: 0%");
    blbl_maintenance.setContent(" Maintenance: 0 / 1");

    mlblbl_activity.setLines(3);
}

rcll_draw::RobotLabel::~RobotLabel(){

}

void rcll_draw::RobotLabel::setGeometry(int x, int y, int w, int h){
    blbl_name.setPos(x, y);
    mlblbl_activity.setPos(x, y + h/5);
    blbl_activetime.setPos(x, y + 3*h/5);
    blbl_maintenance.setPos(x, y + 4*h/5);

    blbl_name.setSize(w, h/5);
    mlblbl_activity.setSize(w, 2*h/5);
    blbl_activetime.setSize(w, h/5);
    blbl_maintenance.setSize(w, h/5);
}

void rcll_draw::RobotLabel::setRobotName(int id, std::string name, bool active){
    if (active){
        blbl_name.setFrontColor(rcll_draw::C_GREEN_LIGHT);
        blbl_name.setContent(" R" + std::to_string(id) + ": " + name);
    } else {
        blbl_name.setFrontColor(rcll_draw::C_RED);
        blbl_name.setContent(" R" + std::to_string(id) + ": inactive");
    }
    this->active = active;
}

void rcll_draw::RobotLabel::setRobotStatus(std::string activity_str, double active_time, int maintenance_count, int maintenance_max){
    if (active){
        if (activity_str != ""){
            mlblbl_activity.setContent("Activity: " + activity_str);
        } else {
            mlblbl_activity.setContent("Activity: unknown");
        }
    } else {
        mlblbl_activity.setContent("Activity: inactive");
    }

    blbl_activetime.setContent(" Active Time: " + std::to_string((int)(active_time * 100.0)) + "%");
    blbl_maintenance.setContent(" Maintenance: " + std::to_string(maintenance_count) + " / " + std::to_string(maintenance_max));
}

void rcll_draw::RobotLabel::draw(cv::Mat &mat){
    blbl_name.draw(mat);
    mlblbl_activity.draw(mat);
    blbl_activetime.draw(mat);
    blbl_maintenance.draw(mat);
}
