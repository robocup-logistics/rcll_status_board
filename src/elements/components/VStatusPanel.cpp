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

#include <VStatusPanel.h>

// VStatusPanel ####################################################################
rcll_draw::VStatusPanel::VStatusPanel() : rcll_draw::VStatusPanel::VStatusPanel(rcll_draw::NO_TEAM){

}

rcll_draw::VStatusPanel::VStatusPanel(rcll_draw::Team team){
    origin = cv::Mat(h, w, CV_8UC4);
    this->team = team;
    blbl_state_header.setContent("STATE");
    blbl_phase_header.setContent("PHASE");
    blbl_time_header.setContent("TIME");
    blbl_score_header.setContent("SCORE");

    blbl_state_header.setAlignment(rcll_draw::CenterCenter);
    blbl_phase_header.setAlignment(rcll_draw::CenterCenter);
    blbl_time_header.setAlignment(rcll_draw::CenterCenter);
    blbl_score_header.setAlignment(rcll_draw::CenterCenter);
    blbl_state_value.setAlignment(rcll_draw::CenterCenter);
    blbl_phase_value.setAlignment(rcll_draw::CenterCenter);
    blbl_time_value.setAlignment(rcll_draw::CenterCenter);
    blbl_score_value_cyan.setAlignment(rcll_draw::CenterCenter);
    blbl_score_value_mid.setAlignment(rcll_draw::CenterCenter);
    blbl_score_value_magenta.setAlignment(rcll_draw::CenterCenter);

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

    blbl_state_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_phase_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_time_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_score_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_state_value.setFrontColor(rcll_draw::C_BLACK);
    blbl_phase_value.setFrontColor(rcll_draw::C_BLACK);
    blbl_time_value.setFrontColor(rcll_draw::C_BLACK);

    blbl_score_value_cyan.setFrontColor(rcll_draw::C_CYAN_DARK);
    blbl_score_value_magenta.setFrontColor(rcll_draw::C_MAGENTA_DARK);
    if (team == rcll_draw::CYAN){
        blbl_score_value_mid.setFrontColor(rcll_draw::C_CYAN_DARK);
    } else if (team == rcll_draw::MAGENTA){
        blbl_score_value_mid.setFrontColor(rcll_draw::C_MAGENTA_DARK);
    } else {
        blbl_score_value_mid.setFrontColor(rcll_draw::C_BLACK);
        blbl_score_value_mid.setContent("/");
    }
}

rcll_draw::VStatusPanel::~VStatusPanel(){

}

void rcll_draw::VStatusPanel::setGeometry(int x, int y, double scale){
    this->x = x;
    this->y = y;
    this->scale = scale;

    blbl_state_header.setPos(0, 0);
    blbl_phase_header.setPos(0, 2 * h / 9);
    blbl_time_header.setPos(0, 4 * h / 9);
    blbl_score_header.setPos(0, 6 * h / 9);
    blbl_state_value.setPos(0, 1 * h / 9);
    blbl_phase_value.setPos(0, 3 * h / 9);
    blbl_time_value.setPos(0, 5 * h / 9);
    blbl_score_value_mid.setPos(0, 7 * h / 9);
    blbl_score_value_cyan.setPos(0, 7 * h / 9);
    blbl_score_value_magenta.setPos(0.6 * w, 7 * h / 9);

    blbl_state_header.setSize(w, h / 9);
    blbl_phase_header.setSize(w, h / 9);
    blbl_time_header.setSize(w, h / 9);
    blbl_score_header.setSize(w, h / 9);
    blbl_state_value.setSize(w, h / 9);
    blbl_phase_value.setSize(w, h / 9);
    blbl_time_value.setSize(w, h / 9);
    blbl_score_value_mid.setSize(w, 2 * h / 9);
    blbl_score_value_cyan.setSize(w * 0.4, 2 * h / 9);
    blbl_score_value_magenta.setSize(w * 0.4, 2 * h / 9);
}

int rcll_draw::VStatusPanel::getW(double scale){
    return (int)((double)w * scale);
}

int rcll_draw::VStatusPanel::getH(double scale){
    return (int)((double)h * scale);
}


void rcll_draw::VStatusPanel::setContent(rcll_vis_msgs::GameInfo &gameinfo){
    int min = gameinfo.phase_time / 60;
    int sec = std::fmod(gameinfo.phase_time, 60);
    std::string time_str = std::to_string(min) + "min " + std::to_string(sec) + "sec";
    blbl_state_value.setContent(rcll_draw::getGameStateStr(gameinfo.game_state));
    blbl_phase_value.setContent(rcll_draw::getGamePhaseStr(gameinfo.game_phase));
    blbl_time_value.setContent(time_str);

    if (team == rcll_draw::CYAN){
        blbl_score_value_mid.setContent(std::to_string(gameinfo.team_points_cyan));
        blbl_score_value_mid.setFontSize(2);
    } else if (team == rcll_draw::MAGENTA){
        blbl_score_value_mid.setContent(std::to_string(gameinfo.team_points_magenta));
        blbl_score_value_mid.setFontSize(2);
    } else {
        blbl_score_value_cyan.setContent(std::to_string(gameinfo.team_points_cyan));
        blbl_score_value_magenta.setContent(std::to_string(gameinfo.team_points_magenta));
        blbl_score_value_mid.setContent("/");

        blbl_score_value_cyan.setFontSize(1.3);
        blbl_score_value_mid.setFontSize(1.3);
        blbl_score_value_magenta.setFontSize(1.3);
    }
}

void rcll_draw::VStatusPanel::draw(cv::Mat &mat, bool show_element_border){
    cv::rectangle(origin, cv::Point(0, 0), cv::Point (w-1, h-1), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED);

    blbl_state_header.draw(origin);
    blbl_phase_header.draw(origin);
    blbl_time_header.draw(origin);
    blbl_score_header.draw(origin);
    blbl_state_value.draw(origin);
    blbl_phase_value.draw(origin);
    blbl_time_value.draw(origin);
    blbl_score_value_mid.draw(origin);
    blbl_score_value_cyan.draw(origin);
    blbl_score_value_magenta.draw(origin);

    if (show_element_border){
        cv::rectangle(origin, cv::Point(0, 0), cv::Point (w-1, h-1), rcll_draw::getColor(rcll_draw::C_RED), 1);
    }

    rcll_draw::mergeImages(mat, origin, x, y, scale);
}
