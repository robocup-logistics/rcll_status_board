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

#include <RobotInfo.h>
// RobotInfo ####################################################################

rcll_draw::RobotInfo::RobotInfo(){

}

rcll_draw::RobotInfo::RobotInfo(Team team){
    blbl_header.setAlignment(rcll_draw::CenterCenter);
    blbl_header.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_header.setFontSize(2.0);
    blbl_header.setContent("ROBOTS");
    if (team == rcll_draw::CYAN){
        blbl_header.setFrontColor(rcll_draw::C_CYAN_DARK);
    } else if (team == rcll_draw::MAGENTA){
        blbl_header.setFrontColor(rcll_draw::C_MAGENTA_DARK);
    } else {
        blbl_header.setFrontColor(rcll_draw::C_BLACK);
    }


    for (size_t i = 0; i < keys.size(); i++){
        RobotLabel rl;
        rcll_vis_msgs::Robot robot;
        robot.key = keys[i];
        rl.setRobot(robot);
        rl_robots.push_back(rl);
    }
}

rcll_draw::RobotInfo::~RobotInfo(){

}

void rcll_draw::RobotInfo::setGeometry(int x, int y, int w, int h, int gapsize){
    int w1 = (w - 2 * gapsize) / 3;
    blbl_header.setPos(x, y);
    blbl_header.setSize(w, h*0.2);

    for (size_t i = 0; i < rl_robots.size(); i++){
        rl_robots[i].setGeometry(x + i * (gapsize + w1), y + h*0.2, w1, h*0.8);
    }

    ROS_INFO("RobotInfo w=%i h=%i", w, h);
}

void rcll_draw::RobotInfo::setRobots(std::vector<rcll_vis_msgs::Robot> &robots){
    for (size_t i = 0; i < robots.size(); i++){
        if (team == (rcll_draw::Team)robots[i].team){
            rl_robots[robot_map[robots[i].key]].setRobot(robots[i]);
        }
    }
}

void rcll_draw::RobotInfo::draw(cv::Mat &mat){
    blbl_header.draw(mat);
    for (size_t i = 0; i < rl_robots.size(); i++){
        rl_robots[i].draw(mat);
    }
}
