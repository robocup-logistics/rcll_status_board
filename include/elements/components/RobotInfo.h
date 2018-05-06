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

#ifndef RCLL_ROBOT_INFO_H
#define RCLL_ROBOT_INFO_H

#include <util.h>
#include <BoxLabel.h>
#include <RobotLabel.h>

#include <rcll_vis_msgs/Robots.h>

namespace rcll_draw {
    class RobotInfo {
    public:
        RobotInfo();
        RobotInfo(Team team);
        ~RobotInfo();

        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setRobots(std::vector<rcll_vis_msgs::Robot> &robots);
        void draw(cv::Mat &mat);

    private:
        Team team;
        BoxLabel blbl_header;

        std::vector<std::string> keys = {"Rob1", "Rob2", "Rob3"};
        std::map<std::string, size_t> robot_map;
        std::vector<RobotLabel> rl_robots;
    };
}

#endif
