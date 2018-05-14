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

#ifndef RCLL_AREA_PRODUCTION_H
#define RCLL_AREA_PRODUCTION_H

#include <util.h>

#include <HeaderPanel.h>
#include <TeamHeaderPanel.h>
#include <VStatusPanel.h>
#include <ProductInfo.h>
#include <MachineInfoProduction.h>
#include <RobotInfo.h>
#include <GameField.h>

#include <rcll_vis_msgs/GameInfo.h>
#include <rcll_vis_msgs/Machine.h>
#include <rcll_vis_msgs/Robot.h>
#include <rcll_vis_msgs/Product.h>

namespace rcll_draw {
    class AreaProduction {
    public:
        AreaProduction();
        ~AreaProduction();

        void setGeometry(int x, int y, int w, int h);
        void setGameInfo(rcll_vis_msgs::GameInfo &gameinfo);
        void setMachines(std::vector<rcll_vis_msgs::Machine> &machines);
        void setRobots(std::vector<rcll_vis_msgs::Robot> &robots);
        void setGameField(rcll_vis_msgs::SetGameField &setgamefield);
        void setProducts(std::vector<rcll_vis_msgs::Product> &products);
        void setPaging(double paging_time, double paging_wait_time, int shift_increase);
        void setRefBoxView(bool refbox_view);
        void draw(cv::Mat &mat, bool show_element_borders = false);

    private:
        int x, y = 0;
        int w, h = 1;
        int gapsize = 40;

        Team team;

        int paging_count = 0;
        double paging_time = 5.0;
        ros::Time last_paging;

        HeaderPanel hp_header;
        TeamHeaderPanel thp_team_header_cyan;
        TeamHeaderPanel thp_team_header_magenta;
        VStatusPanel vsp_gameinfo;
        ProductInfo pi_productinfo_cyan;
        ProductInfo pi_productinfo_magenta;
        MachineInfoProduction mip_machineinfo_cyan;
        MachineInfoProduction mip_machineinfo_magenta;
        RobotInfo ri_robotinfo_cyan;
        RobotInfo ri_robotinfo_magenta;
        GameField gf_gamefield;
        BoxLabel blbl_text;
    };
}

#endif
