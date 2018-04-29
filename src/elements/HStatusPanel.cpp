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

#include <HStatusPanel.h>

// HStatusPanel ####################################################################

rcll_draw::HStatusPanel::HStatusPanel(){
    blbl_state_header.setContent("STATE");
    blbl_phase_header.setContent("PHASE");
    blbl_time_header.setContent("TIME");
    blbl_score_header.setContent("SCORE");
    blbl_score_value_mid.setContent("/");

    blbl_state_header.setAlignment(rcll_draw::CenterCenter);
    blbl_phase_header.setAlignment(rcll_draw::CenterCenter);
    blbl_time_header.setAlignment(rcll_draw::CenterCenter);
    blbl_score_header.setAlignment(rcll_draw::CenterCenter);
    blbl_state_value.setAlignment(rcll_draw::CenterCenter);
    blbl_phase_value.setAlignment(rcll_draw::CenterCenter);
    blbl_time_value.setAlignment(rcll_draw::CenterCenter);
    blbl_score_value_cyan.setAlignment(rcll_draw::CenterRight);
    blbl_score_value_mid.setAlignment(rcll_draw::CenterCenter);
    blbl_score_value_magenta.setAlignment(rcll_draw::CenterLeft);

    blbl_state_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_phase_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_time_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_score_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_state_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_phase_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_time_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_score_value_cyan.setBackgroundColor(rcll_draw::C_TRANSPARENT);
    blbl_score_value_mid.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_score_value_magenta.setBackgroundColor(rcll_draw::C_TRANSPARENT);

    blbl_state_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_phase_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_time_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_score_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_state_value.setBorderColor(rcll_draw::C_WHITE);
    blbl_phase_value.setBorderColor(rcll_draw::C_WHITE);
    blbl_time_value.setBorderColor(rcll_draw::C_WHITE);
    blbl_score_value_cyan.setBorderColor(rcll_draw::C_TRANSPARENT);
    blbl_score_value_mid.setBorderColor(rcll_draw::C_WHITE);
    blbl_score_value_magenta.setBorderColor(rcll_draw::C_TRANSPARENT);

    blbl_state_header.setFontSize(1.0);
    blbl_phase_header.setFontSize(1.0);
    blbl_time_header.setFontSize(1.0);
    blbl_score_header.setFontSize(1.0);
    blbl_state_value.setFontSize(0.9);
    blbl_phase_value.setFontSize(0.9);
    blbl_time_value.setFontSize(0.9);
    blbl_score_value_cyan.setFontSize(1.0);
    blbl_score_value_mid.setFontSize(1.0);
    blbl_score_value_magenta.setFontSize(1.0);

    blbl_state_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_phase_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_time_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_score_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_state_value.setFrontColor(rcll_draw::C_BLACK);
    blbl_phase_value.setFrontColor(rcll_draw::C_BLACK);
    blbl_time_value.setFrontColor(rcll_draw::C_BLACK);
    blbl_score_value_cyan.setFrontColor(rcll_draw::C_CYAN_DARK);
    blbl_score_value_mid.setFrontColor(rcll_draw::C_BLACK);
    blbl_score_value_magenta.setFrontColor(rcll_draw::C_MAGENTA_DARK);
}

rcll_draw::HStatusPanel::~HStatusPanel(){

}

void rcll_draw::HStatusPanel::setGeometry(int x, int y, int w, int h){
    blbl_state_header.setPos(x, y);
    blbl_phase_header.setPos(x + w / 4, y);
    blbl_time_header.setPos(x + 2 * w / 4, y);
    blbl_score_header.setPos(x + 3 * w / 4, y);
    blbl_state_value.setPos(x, y + h / 2);
    blbl_phase_value.setPos(x + w / 4, y + h / 2);
    blbl_time_value.setPos(x + 2 * w / 4, y + h / 2);
    blbl_score_value_mid.setPos(x + 3 * w / 4, y + h / 2);
    int w1 = w / 4;
    blbl_score_value_cyan.setPos(x + 3 * w / 4, y + h / 2);
    blbl_score_value_magenta.setPos(x + 3 * w / 4 + w1 *0.6, y + h / 2);

    blbl_state_header.setSize(w / 4, h / 2);
    blbl_phase_header.setSize(w / 4, h / 2);
    blbl_time_header.setSize(w / 4, h / 2);
    blbl_score_header.setSize(w / 4, h / 2);
    blbl_state_value.setSize(w / 4, h / 2);
    blbl_phase_value.setSize(w / 4, h / 2);
    blbl_time_value.setSize(w / 4, h / 2);
    blbl_score_value_mid.setSize(w / 4, h / 2);
    blbl_score_value_cyan.setSize(w1 * 0.4, h / 2);
    blbl_score_value_magenta.setSize(w1 * 0.4, h / 2);
}

void rcll_draw::HStatusPanel::setContent(rcll_vis_msgs::GameInfo &gameinfo){
    int min = gameinfo.phase_time / 60;
    int sec = std::fmod(gameinfo.phase_time, 60);
    std::string time_str = std::to_string(min) + "min " + std::to_string(sec) + "sec";
    blbl_state_value.setContent(rcll_draw::getGameStateStr(gameinfo.game_state));
    blbl_phase_value.setContent(rcll_draw::getGamePhaseStr(gameinfo.game_phase));
    blbl_time_value.setContent(time_str);
    blbl_score_value_cyan.setContent(std::to_string(gameinfo.team_points_cyan));
    blbl_score_value_magenta.setContent(std::to_string(gameinfo.team_points_magenta));
}

void rcll_draw::HStatusPanel::draw(cv::Mat &mat){
    blbl_state_header.draw(mat);
    blbl_phase_header.draw(mat);
    blbl_time_header.draw(mat);
    blbl_score_header.draw(mat);
    blbl_state_value.draw(mat);
    blbl_phase_value.draw(mat);
    blbl_time_value.draw(mat);
    blbl_score_value_mid.draw(mat);
    blbl_score_value_cyan.draw(mat);
    blbl_score_value_magenta.draw(mat);
}
