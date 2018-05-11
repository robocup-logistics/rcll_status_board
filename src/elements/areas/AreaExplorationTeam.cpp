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

#include <AreaExplorationTeam.h>

// AreaExplorationTeam ####################################################################

rcll_draw::AreaExplorationTeam::AreaExplorationTeam() : rcll_draw::AreaExplorationTeam::AreaExplorationTeam(rcll_draw::NO_TEAM){

}

rcll_draw::AreaExplorationTeam::AreaExplorationTeam(rcll_draw::Team team){
    if (team == rcll_draw::CYAN){
        hp_header = HeaderPanel("CYAN STATUS BOARD", team);
    } else if (team == rcll_draw::CYAN){
        hp_header = HeaderPanel("MAGENTA STATUS BOARD", team);
    } else {
        hp_header = HeaderPanel("LOGISTICS LEAGUE - EXPLORATION", team);
    }
    hsp_gameinfo = HStatusPanel();
    mie_machine_info = MachineInfoExploration(team);
}

rcll_draw::AreaExplorationTeam::~AreaExplorationTeam(){

}

void rcll_draw::AreaExplorationTeam::setGeometry(int x, int y, int w, int h){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;

    int cur_y = y;

    hp_header.setGeometry(x + (w - hp_header.getW(1.0)) / 2, cur_y, 1.0);
    cur_y += hp_header.getH(1.0) + gapsize;

    hsp_gameinfo.setGeometry(x + (w - hsp_gameinfo.getW(1.0)) / 2, cur_y, 1.0);
    cur_y += hsp_gameinfo.getH(1.0) + gapsize;

    mie_machine_info.setGeometry(x + (w - mie_machine_info.getW(1.0)) / 2, cur_y, 1.0);
}

void rcll_draw::AreaExplorationTeam::setGameInfo(rcll_vis_msgs::GameInfo &gameinfo){
    hsp_gameinfo.setContent(gameinfo);
}

void rcll_draw::AreaExplorationTeam::setMachines(std::vector<rcll_vis_msgs::Machine> &machines){
    mie_machine_info.setMachines(machines);
}

void rcll_draw::AreaExplorationTeam::draw(cv::Mat &mat, bool show_element_borders){
    hp_header.draw(mat, show_element_borders);
    hsp_gameinfo.draw(mat, show_element_borders);
    mie_machine_info.draw(mat, show_element_borders);
}
