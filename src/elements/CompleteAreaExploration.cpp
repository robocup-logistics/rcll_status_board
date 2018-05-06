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

#include <CompleteAreaExploration.h>

// CompleteAreaExploration ####################################################################

rcll_draw::CompleteAreaExploration::CompleteAreaExploration(){

}

rcll_draw::CompleteAreaExploration::CompleteAreaExploration(rcll_draw::Team team){
    hsp_gameinfo = HStatusPanel();
    thp_team_header_cyan = TeamHeaderPanel();
    thp_team_header_magenta = TeamHeaderPanel();
    mie_machine_info_cyan = MachineInfoExploration(rcll_draw::CYAN);
    mie_machine_info_magenta = MachineInfoExploration(rcll_draw::MAGENTA);
    gf_gamefield = GameField();
}

rcll_draw::CompleteAreaExploration::~CompleteAreaExploration(){

}

void rcll_draw::CompleteAreaExploration::setGeometry(int x, int y, int w, int h, int gapsize){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
    hsp_gameinfo.setGeometry(x + w * 0.2, y, w * 0.6, h * 0.1 - gapsize);
    gf_gamefield.setGeometry(x + w * 0.15, y + h * 0.1 + gapsize, w * 0.7, h * 0.78 - gapsize);
    thp_team_header_cyan.setGeometry(x, y, w * 0.15, h * 0.2);
    thp_team_header_magenta.setGeometry(x + w * 0.85, y + h * 0.1 + gapsize, w * 0.15, h * 0.2);
    mie_machine_info_cyan.setGeometry(x, y, w * 0.15, h * 0.78 - gapsize, gapsize);
    mie_machine_info_magenta.setGeometry(x + w * 0.85, y + h * 0.1 + gapsize, w * 0.15, h * 0.78 - gapsize, gapsize);
}

void rcll_draw::CompleteAreaExploration::setGameInfo(rcll_vis_msgs::GameInfo &gameinfo){
    hsp_gameinfo.setContent(gameinfo);
    thp_team_header_cyan.setTeam(gameinfo.team_name_cyan, rcll_draw::CYAN);
    thp_team_header_magenta.setTeam(gameinfo.team_name_magenta, rcll_draw::MAGENTA);
}

void rcll_draw::CompleteAreaExploration::setRefBoxView(bool refbox_view){
    gf_gamefield.setRefBoxView(refbox_view);
}

void rcll_draw::CompleteAreaExploration::setMachines(std::vector<rcll_vis_msgs::Machine> &machines){
    gf_gamefield.setMachines(machines);
    mie_machine_info_cyan.setMachines(machines);
    mie_machine_info_magenta.setMachines(machines);
}

void rcll_draw::CompleteAreaExploration::setGameField(rcll_vis_msgs::SetGameField &setgamefield){
    gf_gamefield.setGameField(setgamefield);
}

void rcll_draw::CompleteAreaExploration::setRobots(std::vector<rcll_vis_msgs::Robot> &robots){
    return gf_gamefield.setRobots(robots);
}

void rcll_draw::CompleteAreaExploration::draw(cv::Mat &mat){
    hsp_gameinfo.draw(mat);
    gf_gamefield.draw(mat);
    mie_machine_info_cyan.draw(mat);
    mie_machine_info_magenta.draw(mat);
    thp_team_header_cyan.draw(mat);
    thp_team_header_magenta.draw(mat);
}
