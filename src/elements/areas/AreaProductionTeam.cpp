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

#include <AreaProductionTeam.h>

// AreaProductionTeam ####################################################################

rcll_draw::AreaProductionTeam::AreaProductionTeam(){
    hp_header = HeaderPanel("STATUS BOARD", rcll_draw::NO_TEAM);
    vsp_gameinfo = VStatusPanel(rcll_draw::NO_TEAM);
    pi_productinfo = ProductInfo(rcll_draw::NO_TEAM);
    mip_machineinfo = MachineInfoProduction(rcll_draw::NO_TEAM);
    ri_robotinfo = RobotInfo(rcll_draw::NO_TEAM);
}

rcll_draw::AreaProductionTeam::AreaProductionTeam(rcll_draw::Team team){
    if (team == rcll_draw::CYAN){
        hp_header = HeaderPanel("CYAN STATUS BOARD", team);
    } else if (team == rcll_draw::CYAN){
        hp_header = HeaderPanel("MAGENTA STATUS BOARD", team);
    } else {
        hp_header = HeaderPanel("STATUS BOARD", team);
    }
    vsp_gameinfo = VStatusPanel(team);
    pi_productinfo = ProductInfo(team);
    mip_machineinfo = MachineInfoProduction(team);
    ri_robotinfo = RobotInfo(team);
}

rcll_draw::AreaProductionTeam::~AreaProductionTeam(){

}

void rcll_draw::AreaProductionTeam::setGeometry(int x, int y, int w, int h, int gapsize){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;

    int cur_y = y;

    hp_header.setGeometry(x + (w - hp_header.getW(1.0)) / 2, cur_y, 1.0);
    cur_y += hp_header.getH(1.0) + gapsize;

    vsp_gameinfo.setGeometry(x, cur_y, 1.0);
    pi_productinfo.setGeometry(x + w - pi_productinfo.getW(0.96), cur_y, 0.96);
    cur_y += vsp_gameinfo.getH(1.0) + gapsize;

    mip_machineinfo.setGeometry(x, cur_y, 1.0);
    ri_robotinfo.setGeometry(x + w - ri_robotinfo.getW(), cur_y, 1.0);
}

void rcll_draw::AreaProductionTeam::setGameInfo(rcll_vis_msgs::GameInfo &gameinfo){
    vsp_gameinfo.setContent(gameinfo);
}

void rcll_draw::AreaProductionTeam::setMachines(std::vector<rcll_vis_msgs::Machine> &machines){
    mip_machineinfo.setMachines(machines);
}

void rcll_draw::AreaProductionTeam::setRobots(std::vector<rcll_vis_msgs::Robot> &robots){
    ri_robotinfo.setRobots(robots);
}

void rcll_draw::AreaProductionTeam::setProducts(std::vector<rcll_vis_msgs::Product> &products){
    pi_productinfo.setProducts(products);
}

void rcll_draw::AreaProductionTeam::draw(cv::Mat &mat, bool show_element_borders){
    hp_header.draw(mat, show_element_borders);
    vsp_gameinfo.draw(mat, show_element_borders);
    pi_productinfo.draw(mat, show_element_borders);
    mip_machineinfo.draw(mat, show_element_borders);
    ri_robotinfo.draw(mat, show_element_borders);
}
