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

#include <rcll_vis_msgs/SetGameField.h>
#include <rcll_vis_msgs/Machine.h>
#include <rcll_vis_msgs/Robot.h>

namespace rcll_draw {

    // ##################################################

    class GameField {
    public:
        GameField();
        ~GameField();

        void setPhase(rcll_draw::GamePhase gamephase);
        void setGeometry(int x, int y, int w, int h);
        void setGameField(rcll_vis_msgs::SetGameField &setgamefield);
        void setRefBoxView(bool refbox_view);

        void setRobots(std::vector<rcll_vis_msgs::Robot> &robots);
        void setMachines(std::vector<rcll_vis_msgs::Machine> &machines);
        void draw(cv::Mat &mat);

    private:
        int x, y = 0;
        int w, h = 1;

        int x0, y0 = 0;
        int origin_x, origin_y = 0;
        int w0, h0 = 1;

        bool refbox_view;
        int pixel_per_meter = 10;

        GamePhase gamephase;
        Rectangle rct_background;
        Rectangle rct_background2;
        BoxLabel blbl_insertion_cyan1;
        BoxLabel blbl_insertion_cyan2;
        BoxLabel blbl_insertion_magenta1;
        BoxLabel blbl_insertion_magenta2;
        std::vector<Line> lnes_walls;
        std::vector<Line> lnes_zone_lines;
        std::vector<BoxLabel> blbls_zone_names;

        std::vector<RobotMarker> rm_robot_markers;
        std::vector<MachineMarker> mm_machine_markers;

    };
}

#endif
