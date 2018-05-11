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

#include <AreaProduction.h>

// AreaProduction ####################################################################

rcll_draw::AreaProduction::AreaProduction(){
    hp_header = HeaderPanel("LOGISTICS LEAGUE - PRODUCTION", rcll_draw::NO_TEAM);
    vsp_gameinfo = VStatusPanel(rcll_draw::NO_TEAM);
    pi_productinfo_cyan = ProductInfo(rcll_draw::CYAN, 3);
    pi_productinfo_magenta = ProductInfo(rcll_draw::MAGENTA, 3);
    mip_machineinfo_cyan = MachineInfoProduction(rcll_draw::CYAN);
    mip_machineinfo_magenta = MachineInfoProduction(rcll_draw::MAGENTA);
    ri_robotinfo_cyan = RobotInfo(rcll_draw::CYAN);
    ri_robotinfo_magenta = RobotInfo(rcll_draw::MAGENTA);
    gf_gamefield = GameField();

    mip_machineinfo_cyan.setShortDisplay(true);
    mip_machineinfo_magenta.setShortDisplay(true);
    gf_gamefield.setRefBoxView(false);
}

rcll_draw::AreaProduction::~AreaProduction(){

}

void rcll_draw::AreaProduction::setGeometry(int x, int y, int w, int h){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;

    int cur_y = y;

    hp_header.setGeometry(x + (w - hp_header.getW(1.0)) / 2, cur_y, 1.0);
    cur_y += hp_header.getH(1.0) + gapsize;

    vsp_gameinfo.setGeometry(x + (w - vsp_gameinfo.getW(0.71)) / 2, cur_y, 0.71);
    pi_productinfo_cyan.setGeometry(x, cur_y, 0.65);
    pi_productinfo_magenta.setGeometry(x + w - pi_productinfo_magenta.getW(0.65), cur_y, 0.65);
    ri_robotinfo_cyan.setGeometry(x, cur_y, 1.0);
    ri_robotinfo_magenta.setGeometry(x + w - ri_robotinfo_magenta.getW(1.0), cur_y, 1.0);
    cur_y += vsp_gameinfo.getH(0.71) + gapsize;

    mip_machineinfo_cyan.setGeometry(x, cur_y, 0.85);
    mip_machineinfo_magenta.setGeometry(x + w - mip_machineinfo_magenta.getW(0.85), cur_y, 0.85);
    gf_gamefield.setGeometry(x + (w - gf_gamefield.getW(0.68)) / 2, cur_y, 0.68);
}

void rcll_draw::AreaProduction::setGameInfo(rcll_vis_msgs::GameInfo &gameinfo){
    vsp_gameinfo.setContent(gameinfo);
    gf_gamefield.setPhase((rcll_draw::GamePhase)gameinfo.game_phase);
}

void rcll_draw::AreaProduction::setMachines(std::vector<rcll_vis_msgs::Machine> &machines){
    mip_machineinfo_cyan.setMachines(machines);
    mip_machineinfo_magenta.setMachines(machines);
    gf_gamefield.setMachines(machines);
}

void rcll_draw::AreaProduction::setRobots(std::vector<rcll_vis_msgs::Robot> &robots){
    ri_robotinfo_cyan.setRobots(robots);
    ri_robotinfo_magenta.setRobots(robots);
    gf_gamefield.setRobots(robots);
}

void rcll_draw::AreaProduction::setGameField(rcll_vis_msgs::SetGameField &setgamefield){
    gf_gamefield.setGameField(setgamefield);
}


void rcll_draw::AreaProduction::setRefBoxView(bool refbox_view){
    gf_gamefield.setRefBoxView(refbox_view);
}

void rcll_draw::AreaProduction::setProducts(std::vector<rcll_vis_msgs::Product> &products){
    pi_productinfo_cyan.setProducts(products);
    pi_productinfo_magenta.setProducts(products);
}

void rcll_draw::AreaProduction::setPagingTime(double paging_time){
    this->paging_time = paging_time;
    last_paging = ros::Time::now();
}

void rcll_draw::AreaProduction::draw(cv::Mat &mat, bool show_element_borders){
    hp_header.draw(mat, show_element_borders);
    vsp_gameinfo.draw(mat, show_element_borders);

    bool allow_paging_by_move = true;

    if (paging_count % 2 == 0){
        pi_productinfo_cyan.draw(mat, show_element_borders);
        pi_productinfo_magenta.draw(mat, show_element_borders);
        allow_paging_by_move &= !pi_productinfo_cyan.move();
        allow_paging_by_move &= !pi_productinfo_magenta.move();
    } else {
        allow_paging_by_move = true;
        ri_robotinfo_cyan.draw(mat, show_element_borders);
        ri_robotinfo_magenta.draw(mat, show_element_borders);
    }

    if (allow_paging_by_move && (ros::Time::now() - last_paging).toSec() >= paging_time){
        paging_count++;
        last_paging = ros::Time::now();
    }

    mip_machineinfo_cyan.draw(mat, show_element_borders);
    mip_machineinfo_magenta.draw(mat, show_element_borders);
    gf_gamefield.draw(mat, show_element_borders);
}
