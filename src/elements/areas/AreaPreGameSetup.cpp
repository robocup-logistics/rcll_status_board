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

#include <AreaPreGameSetup.h>

// AreaPreGameSetup ####################################################################

rcll_draw::AreaPreGameSetup::AreaPreGameSetup() : rcll_draw::AreaPreGameSetup::AreaPreGameSetup(rcll_draw::NO_TEAM){

}

rcll_draw::AreaPreGameSetup::AreaPreGameSetup(rcll_draw::Team team){
    if (team == rcll_draw::CYAN){
        hp_header = HeaderPanel("CYAN STATUS BOARD", team);
    } else if (team == rcll_draw::CYAN){
        hp_header = HeaderPanel("MAGENTA STATUS BOARD", team);
    } else {
        hp_header = HeaderPanel("STATUS BOARD", team);
    }

    blbl_versus.setContent("versus");
    blbl_text.setContent("Last preparations are arranged by the teams. The game will start shortly.");

    blbl_versus.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_text.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_versus.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_text.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_versus.setBorderColor(rcll_draw::C_WHITE);
    blbl_text.setBorderColor(rcll_draw::C_WHITE);

    blbl_versus.setFontSize(1.0);
    blbl_text.setFontSize(1.0);
}

rcll_draw::AreaPreGameSetup::~AreaPreGameSetup(){

}

void rcll_draw::AreaPreGameSetup::setGameInfo(rcll_vis_msgs::GameInfo &gameinfo){
    hsp_gameinfo.setContent(gameinfo);
    thp_team_cyan.setTeam(gameinfo.team_name_cyan, rcll_draw::CYAN);
    thp_team_magenta.setTeam(gameinfo.team_name_magenta, rcll_draw::MAGENTA);
}

void rcll_draw::AreaPreGameSetup::setGeometry(int x, int y, int w, int h){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;

    blbl_versus.setSize(w * 0.1, h * 0.1 / 3);
    blbl_text.setSize(w * 0.8, h * 0.1);

    int cur_y = y;

    hp_header.setGeometry(x + (w - hp_header.getW(1.0)) / 2, cur_y, 1.0);
    cur_y += hp_header.getH(1.0) + gapsize;

    hsp_gameinfo.setGeometry(x + (w - hsp_gameinfo.getW(1.0)) / 2, cur_y, 1.0);
    cur_y += hsp_gameinfo.getH(1.0) + gapsize;

    thp_team_cyan.setGeometry(x + 0.3 * w - thp_team_cyan.getW(1.0) / 2, cur_y, 1.0);
    thp_team_magenta.setGeometry(x + 0.7 * w - thp_team_magenta.getW(1.0) / 2, cur_y, 1.0);
    blbl_versus.setPos(x + (w - blbl_versus.getW()) / 2, cur_y + blbl_versus.getH() / 2);
    cur_y += thp_team_cyan.getH(1.0) + 2 * gapsize;

    blbl_text.setPos(x + (w - blbl_text.getW()) / 2, cur_y);
}

void rcll_draw::AreaPreGameSetup::draw(cv::Mat &mat, bool show_element_borders){
    hp_header.draw(mat, show_element_borders);
    thp_team_cyan.draw(mat, show_element_borders);
    thp_team_magenta.draw(mat, show_element_borders);
    blbl_versus.draw(mat);
    blbl_text.draw(mat);
    hsp_gameinfo.draw(mat, show_element_borders);
}
