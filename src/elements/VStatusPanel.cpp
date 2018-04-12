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
rcll_draw::VStatusPanel::VStatusPanel(){
    this->team = rcll_draw::NO_TEAM;
}


rcll_draw::VStatusPanel::VStatusPanel(rcll_draw::Team team){
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
    blbl_score_value.setAlignment(rcll_draw::CenterCenter);

    blbl_state_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_phase_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_time_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_score_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_state_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_phase_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_time_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_score_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);

    blbl_state_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_phase_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_time_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_score_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_state_value.setBorderColor(rcll_draw::C_WHITE);
    blbl_phase_value.setBorderColor(rcll_draw::C_WHITE);
    blbl_time_value.setBorderColor(rcll_draw::C_WHITE);
    blbl_score_value.setBorderColor(rcll_draw::C_WHITE);

    blbl_state_header.setFontSize(1.0);
    blbl_phase_header.setFontSize(1.0);
    blbl_time_header.setFontSize(1.0);
    blbl_score_header.setFontSize(1.0);
    blbl_state_value.setFontSize(0.9);
    blbl_phase_value.setFontSize(0.9);
    blbl_time_value.setFontSize(0.9);
    blbl_score_value.setFontSize(2);

    blbl_state_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_phase_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_time_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_score_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_state_value.setFrontColor(rcll_draw::C_BLACK);
    blbl_phase_value.setFrontColor(rcll_draw::C_BLACK);
    blbl_time_value.setFrontColor(rcll_draw::C_BLACK);
    if (team == rcll_draw::CYAN){
        blbl_score_value.setFrontColor(rcll_draw::C_CYAN_DARK);
    } else if (team == rcll_draw::MAGENTA){
        blbl_score_value.setFrontColor(rcll_draw::C_MAGENTA_DARK);
    } else {
        blbl_score_value.setFrontColor(rcll_draw::C_BLACK);
    }
}

rcll_draw::VStatusPanel::~VStatusPanel(){

}

void rcll_draw::VStatusPanel::setGeometry(int x, int y, int w, int h){
    blbl_state_header.setPos(x, y);
    blbl_phase_header.setPos(x, y + 2 * h / 9);
    blbl_time_header.setPos(x, y + 4 * h / 9);
    blbl_score_header.setPos(x, y + 6 * h / 9);
    blbl_state_value.setPos(x, y + 1 * h / 9);
    blbl_phase_value.setPos(x, y + 3 * h / 9);
    blbl_time_value.setPos(x, y + 5 * h / 9);
    blbl_score_value.setPos(x, y + 7 * h / 9);

    blbl_state_header.setSize(w, h / 9);
    blbl_phase_header.setSize(w, h / 9);
    blbl_time_header.setSize(w, h / 9);
    blbl_score_header.setSize(w, h / 9);
    blbl_state_value.setSize(w, h / 9);
    blbl_phase_value.setSize(w, h / 9);
    blbl_time_value.setSize(w, h / 9);
    blbl_score_value.setSize(w, 2 * h / 9);
}

void rcll_draw::VStatusPanel::setContent(std::string gamestate, std::string gamephase, int time, int score){
    int min = time / 60;
    int sec = time % 60;
    std::string time_str = std::to_string(min) + "min " + std::to_string(sec) + "sec";
    blbl_state_value.setContent(gamestate);
    blbl_phase_value.setContent(gamephase);
    blbl_time_value.setContent(time_str);
    blbl_score_value.setContent(std::to_string(score));
}

void rcll_draw::VStatusPanel::draw(cv::Mat &mat){
    blbl_state_header.draw(mat);
    blbl_phase_header.draw(mat);
    blbl_time_header.draw(mat);
    blbl_score_header.draw(mat);
    blbl_state_value.draw(mat);
    blbl_phase_value.draw(mat);
    blbl_time_value.draw(mat);
    blbl_score_value.draw(mat);
}
