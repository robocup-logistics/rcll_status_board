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

#include <TeamAreaPostGame.h>
// TeamAreaPostGame ####################################################################

rcll_draw::TeamAreaPostGame::TeamAreaPostGame(){
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

rcll_draw::TeamAreaPostGame::~TeamAreaPostGame(){

}

void rcll_draw::TeamAreaPostGame::setGameInfo(std::string gamestate, std::string gamephase, int time, int score_cyan, int score_magenta){
    blbl_points_cyan.setContent(std::to_string(score_cyan));
    blbl_points_magenta.setContent(std::to_string(score_magenta));
    game_info.setContent(gamestate, gamephase, time, score_cyan, score_magenta);
}

void rcll_draw::TeamAreaPostGame::setTeams(std::string team_name_cyan, std::string team_name_magenta){
    thpan_team_cyan.setTeam(team_name_cyan, rcll_draw::CYAN);
    thpan_team_magenta.setTeam(team_name_magenta, rcll_draw::MAGENTA);
}

void rcll_draw::TeamAreaPostGame::setGeometry(int x, int y, int w, int h, int gapsize){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;

    thpan_team_cyan.setGeometry(x + w * 0.2, y + h * 0.3, w * 0.2, h * 0.1);
    thpan_team_magenta.setGeometry(x + w * 0.6, y + h * 0.3, w * 0.2, h * 0.1);

    blbl_versus.setSize(w * 0.1, h * 0.1 / 3);
    blbl_versus.setPos(x + w * 0.45, y + h * 0.3 + h * 0.1 * 2 / 3);
    blbl_points.setSize(w * 0.1, h * 0.1 / 3);
    blbl_points.setPos(x + w * 0.45, y + h * 0.45);
    blbl_text.setSize(w * 0.8, h * 0.1);
    blbl_text.setPos(x + w * 0.1, y + h * 0.6);
    blbl_points_cyan.setSize(w * 0.2, h * 0.1);
    blbl_points_cyan.setPos(x + w * 0.2, y + h * 0.42);
    blbl_points_magenta.setSize(w * 0.2, h * 0.1);
    blbl_points_magenta.setPos(x + w * 0.6, y + h * 0.42);

    game_info.setGeometry(x + w * 0.2, y, w * 0.6, h * 0.1);
}

void rcll_draw::TeamAreaPostGame::draw(cv::Mat &mat){
    thpan_team_cyan.draw(mat);
    thpan_team_magenta.draw(mat);
    blbl_versus.draw(mat);
    blbl_text.draw(mat);
    game_info.draw(mat);
    blbl_points.draw(mat);
    blbl_points_cyan.draw(mat);
    blbl_points_magenta.draw(mat);
}
