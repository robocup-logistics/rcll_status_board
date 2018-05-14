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

#include <AreaExploration.h>

// AreaExploration ####################################################################

rcll_draw::AreaExploration::AreaExploration(){
    hp_header = HeaderPanel("LOGISTICS LEAGUE", rcll_draw::NO_TEAM);
    hsp_gameinfo = HStatusPanel();
    thp_team_header_cyan = TeamHeaderPanel();
    thp_team_header_magenta = TeamHeaderPanel();
    mie_machine_info_cyan = MachineInfoExploration(rcll_draw::CYAN);
    mie_machine_info_magenta = MachineInfoExploration(rcll_draw::MAGENTA);
    gf_gamefield = GameField();
    gf_gamefield.setRefBoxView(false);

    blbl_text.setContent("The goal for the robots is to detect and report the machines' positions and orientations.");
    blbl_text.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_text.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_text.setBorderColor(rcll_draw::C_WHITE);
    blbl_text.setFontSize(1.0);
}

rcll_draw::AreaExploration::~AreaExploration(){

}

void rcll_draw::AreaExploration::setGeometry(int x, int y, int w, int h){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;

    mie_machine_info_cyan.setShortDisplay(true);
    mie_machine_info_magenta.setShortDisplay(true);

    int cur_y = y;

    hp_header.setGeometry(x + (w - hp_header.getW(1.0)) / 2, cur_y, 1.0);
    cur_y += hp_header.getH(1.0) + gapsize;

    hsp_gameinfo.setGeometry(x + w * 0.2, cur_y, 1.0);
    cur_y += hsp_gameinfo.getH(1.0) + 2 * gapsize;

    gf_gamefield.setGeometry(x + (w - gf_gamefield.getW(0.8)) / 2, cur_y, 0.8);

    thp_team_header_cyan.setGeometry(x, cur_y, 1.0);
    thp_team_header_magenta.setGeometry(x + w - thp_team_header_magenta.getW(1.0), cur_y, 1.0);
    cur_y += thp_team_header_cyan.getH(1.0) + gapsize;

    mie_machine_info_cyan.setGeometry(x, cur_y, 0.5);
    mie_machine_info_magenta.setGeometry(x + w - mie_machine_info_magenta.getW(0.5), cur_y, 0.5);
    cur_y += mie_machine_info_cyan.getH(0.5) + gapsize;

    blbl_text.setSize(w, h * 0.05);
    blbl_text.setPos(x, cur_y);
}

void rcll_draw::AreaExploration::setGameInfo(rcll_vis_msgs::GameInfo &gameinfo){
    hsp_gameinfo.setContent(gameinfo);
    gf_gamefield.setPhase((rcll_draw::GamePhase)gameinfo.game_phase);
    thp_team_header_cyan.setTeam(gameinfo.team_name_cyan, rcll_draw::CYAN);
    thp_team_header_magenta.setTeam(gameinfo.team_name_magenta, rcll_draw::MAGENTA);
}

void rcll_draw::AreaExploration::setRefBoxView(bool refbox_view){
    gf_gamefield.setRefBoxView(refbox_view);
}

void rcll_draw::AreaExploration::setMachines(std::vector<rcll_vis_msgs::Machine> &machines){
    gf_gamefield.setMachines(machines);
    mie_machine_info_cyan.setMachines(machines);
    mie_machine_info_magenta.setMachines(machines);
}

void rcll_draw::AreaExploration::setGameField(rcll_vis_msgs::SetGameField &setgamefield){
    gf_gamefield.setGameField(setgamefield);
}

void rcll_draw::AreaExploration::setRobots(std::vector<rcll_vis_msgs::Robot> &robots){
    gf_gamefield.setRobots(robots);
}

void rcll_draw::AreaExploration::draw(cv::Mat &mat, bool show_element_borders){
    hp_header.draw(mat, show_element_borders);
    hsp_gameinfo.draw(mat, show_element_borders);
    gf_gamefield.draw(mat, show_element_borders);
    mie_machine_info_cyan.draw(mat, show_element_borders);
    mie_machine_info_magenta.draw(mat, show_element_borders);
    thp_team_header_cyan.draw(mat, show_element_borders);
    thp_team_header_magenta.draw(mat, show_element_borders);
    blbl_text.draw(mat);

    if (show_element_borders){
        cv::rectangle(mat, cv::Point(x, y), cv::Point (x+w-1, y+h-1), rcll_draw::getColor(rcll_draw::C_BLUE), 1);
    }
}
