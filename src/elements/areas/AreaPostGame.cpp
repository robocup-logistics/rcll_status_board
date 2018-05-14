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

#include <AreaPostGame.h>

// AreaPostGame ####################################################################

rcll_draw::AreaPostGame::AreaPostGame() : rcll_draw::AreaPostGame::AreaPostGame(rcll_draw::NO_TEAM){

}

rcll_draw::AreaPostGame::AreaPostGame(rcll_draw::Team team){
    if (team == rcll_draw::CYAN){
        hp_header = HeaderPanel("CYAN STATUS BOARD", team);
    } else if (team == rcll_draw::CYAN){
        hp_header = HeaderPanel("MAGENTA STATUS BOARD", team);
    } else {
        hp_header = HeaderPanel("LOGISTICS LEAGUE", team);
    }

    blbl_versus.setContent("versus");
    blbl_text.setContent("The game has ended. Thank you for watching this match!");
    blbl_points.setContent("Score");

    blbl_versus.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_text.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_points.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_points_cyan.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_points_magenta.setAlignment(rcll_draw::Alignment::CenterCenter);

    blbl_versus.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_text.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_points.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_points_cyan.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_points_magenta.setBackgroundColor(rcll_draw::C_WHITE);

    blbl_versus.setBorderColor(rcll_draw::C_WHITE);
    blbl_text.setBorderColor(rcll_draw::C_WHITE);
    blbl_points.setBorderColor(rcll_draw::C_WHITE);
    blbl_points_cyan.setBorderColor(rcll_draw::C_WHITE);
    blbl_points_magenta.setBorderColor(rcll_draw::C_WHITE);

    blbl_points_cyan.setFrontColor(rcll_draw::C_CYAN_DARK);
    blbl_points_magenta.setFrontColor(rcll_draw::C_MAGENTA_DARK);

    blbl_versus.setFontSize(1.0);
    blbl_text.setFontSize(1.0);
    blbl_points.setFontSize(1.0);
    blbl_points_cyan.setFontSize(2.0);
    blbl_points_magenta.setFontSize(2.0);
}

rcll_draw::AreaPostGame::~AreaPostGame(){

}

void rcll_draw::AreaPostGame::setGameInfo(rcll_vis_msgs::GameInfo &gameinfo){
    thp_team_cyan.setTeam(gameinfo.team_name_cyan, rcll_draw::CYAN);
    thp_team_magenta.setTeam(gameinfo.team_name_magenta, rcll_draw::MAGENTA);
    blbl_points_cyan.setContent(std::to_string(gameinfo.team_points_cyan));
    blbl_points_magenta.setContent(std::to_string(gameinfo.team_points_magenta));
    hsp_gameinfo.setContent(gameinfo);
}

void rcll_draw::AreaPostGame::setGeometry(int x, int y, int w, int h){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;

    blbl_versus.setSize(w * 0.1, h * 0.1 / 3);
    blbl_points.setSize(w * 0.1, h * 0.1 / 3);
    blbl_text.setSize(w * 0.8, h * 0.1);
    blbl_points_cyan.setSize(w * 0.2, h * 0.1);
    blbl_points_magenta.setSize(w * 0.2, h * 0.1);

    int cur_y = y;

    hp_header.setGeometry(x + (w - hp_header.getW(1.0)) / 2, cur_y, 1.0);
    cur_y += hp_header.getH(1.0) + gapsize;

    hsp_gameinfo.setGeometry(x + (w - hsp_gameinfo.getW(1.0)) / 2, cur_y, 1.0);
    cur_y += hsp_gameinfo.getH(1.0) + 3 * gapsize;

    thp_team_cyan.setGeometry(x + 0.3 * w - thp_team_cyan.getW(1.0) / 2, cur_y, 1.0);
    thp_team_magenta.setGeometry(x + 0.7 * w - thp_team_magenta.getW(1.0) / 2, cur_y, 1.0);
    blbl_versus.setPos(x + (w - blbl_versus.getW()) / 2, cur_y + blbl_versus.getH() / 2);
    cur_y += thp_team_cyan.getH(1.0) + gapsize;

    blbl_points_cyan.setPos(x + 0.3 * w - blbl_points_cyan.getW() / 2, cur_y);
    blbl_points_magenta.setPos(x + 0.7 * w - blbl_points_magenta.getW() / 2, cur_y);
    blbl_points.setPos(x + (w - blbl_points.getW()) / 2, cur_y + blbl_points.getH());
    cur_y += blbl_points_cyan.getH() + 2 * gapsize;

    blbl_text.setPos(x + (w - blbl_text.getW()) / 2, cur_y);
}

void rcll_draw::AreaPostGame::draw(cv::Mat &mat, bool show_element_borders){
    hp_header.draw(mat, show_element_borders);
    thp_team_cyan.draw(mat, show_element_borders);
    thp_team_magenta.draw(mat, show_element_borders);
    blbl_versus.draw(mat);
    blbl_text.draw(mat);
    hsp_gameinfo.draw(mat, show_element_borders);
    blbl_points.draw(mat);
    blbl_points_cyan.draw(mat);
    blbl_points_magenta.draw(mat);
}
