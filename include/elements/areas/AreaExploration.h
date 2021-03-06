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

#ifndef RCLL_AREA_EXPLORATION_H
#define RCLL_AREA_EXPLORATION_H

#include <util.h>
#include <HeaderPanel.h>
#include <HStatusPanel.h>
#include <TeamHeaderPanel.h>
#include <MachineInfoExploration.h>
#include <GameField.h>

#include <rcll_vis_msgs/Machine.h>
#include <rcll_vis_msgs/Robot.h>
#include <rcll_vis_msgs/Products.h>
#include <rcll_vis_msgs/GameInfo.h>
#include <rcll_vis_msgs/SetGameField.h>

namespace rcll_draw {
    class AreaExploration {
    public:
        AreaExploration();
        ~AreaExploration();

        void setGeometry(int x, int y, int w, int h);
        void setGameInfo(rcll_vis_msgs::GameInfo &gameinfo);
        void setMachines(std::vector<rcll_vis_msgs::Machine> &machines);
        void setRobots(std::vector<rcll_vis_msgs::Robot> &robots);
        void setGameField(rcll_vis_msgs::SetGameField &setgamefield);
        void setRefBoxView(bool refbox_view);

        void draw(cv::Mat &mat, bool show_element_borders = false);

    private:
        int x, y = 0;
        int w, h = 1;
        int gapsize = 40;

        HeaderPanel hp_header;
        HStatusPanel hsp_gameinfo;
        TeamHeaderPanel thp_team_header_cyan;
        TeamHeaderPanel thp_team_header_magenta;
        MachineInfoExploration mie_machine_info_cyan;
        MachineInfoExploration mie_machine_info_magenta;
        GameField gf_gamefield;
        BoxLabel blbl_text;
    };
}

#endif
