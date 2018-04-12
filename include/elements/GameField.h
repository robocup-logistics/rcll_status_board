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

#ifndef RCLL_GAMEFIELD_H
#define RCLL_GAMEFIELD_H

#include <util.h>
#include <Rectangle.h>
#include <BoxLabel.h>
#include <Line.h>

#include <RobotMarker.h>
#include <MachineMarker.h>

namespace rcll_draw {

    // ##################################################

    class GameField {
    public:
        GameField();
        ~GameField();

        void setPhase(rcll_draw::GamePhase gamephase);
        void setGeometry(int x, int y, int w, int h);
        void setLayout(double field_w, double field_h, int zones_x, int zones_y, std::vector<int> insertion_zones);
        void addWall(double x1, double y1, double x2, double y2);

        size_t addRobot(std::string name, int id, rcll_draw::Team team);
        void setRobotPos(double x, double y, double yaw, size_t index);
        void setMachine(std::string name, rcll_draw::Team team, size_t index);
        void setMachinePos(double x, double y, double yaw, size_t index);
        void setMachineReport(int report1_status, int report2_status, size_t index);
        void draw(cv::Mat &mat);

    private:
        int x, y = 0;
        int w, h = 1;

        int x0, y0 = 0;
        int w0, h0 = 1;

        int pixel_per_meter = 10;

        GamePhase gamephase;
        Rectangle background;
        Rectangle background2;
        BoxLabel insertion_cyan1;
        BoxLabel insertion_cyan2;
        BoxLabel insertion_magenta1;
        BoxLabel insertion_magenta2;
        std::vector<Line> walls;
        std::vector<Line> zone_lines;
        std::vector<BoxLabel> zone_names;

        std::vector<RobotMarker> robot_markers;
        std::vector<MachineMarker> machine_markers;

    };
}

#endif
