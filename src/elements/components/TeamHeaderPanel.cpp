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

#include <TeamHeaderPanel.h>

// TeamHeaderPanel ####################################################################
rcll_draw::TeamHeaderPanel::TeamHeaderPanel(){

    origin = cv::Mat(h, w, CV_8UC4);
}

rcll_draw::TeamHeaderPanel::~TeamHeaderPanel(){

}

void rcll_draw::TeamHeaderPanel::setTeam(std::string team_name, rcll_draw::Team team_color){
    if (team_color == rcll_draw::CYAN){
        blbl_header_color.setContent("CYAN");
        blbl_header_color.setFrontColor(rcll_draw::C_CYAN_DARK);
    } else if (team_color == rcll_draw::MAGENTA){
        blbl_header_color.setContent("MAGENTA");
        blbl_header_color.setFrontColor(rcll_draw::C_MAGENTA_DARK);
    } else {
        blbl_header_color.setContent("");
        blbl_header_color.setFrontColor(rcll_draw::C_BLACK);
    }
    blbl_header_name.setContent(team_name);

    blbl_header_color.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_header_name.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_header_color.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_header_name.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_header_color.setBorderColor(rcll_draw::C_WHITE);
    blbl_header_name.setBorderColor(rcll_draw::C_WHITE);
}

void rcll_draw::TeamHeaderPanel::setGeometry(int x, int y, double scale){
    this->x = x;
    this->y = y;
    this->scale = scale;

    blbl_header_name.setPos(0, 0);
    blbl_header_color.setPos(0, 3 * h / 5);

    blbl_header_name.setSize(w, 3 * h / 5);
    blbl_header_color.setSize(w, 2 * h / 5);

    blbl_header_name.setFontSize(1.7);
    blbl_header_color.setFontSize(1.0);
}

int rcll_draw::TeamHeaderPanel::getW(double scale){
    return (int)((double)w * scale);
}

int rcll_draw::TeamHeaderPanel::getH(double scale){
    return (int)((double)h * scale);
}

void rcll_draw::TeamHeaderPanel::draw(cv::Mat &mat, bool show_element_border){
    cv::rectangle(origin, cv::Point(0, 0), cv::Point (w-1, h-1), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED);

    blbl_header_color.draw(origin);
    blbl_header_name.draw(origin);

    if (show_element_border){
        cv::rectangle(origin, cv::Point(0, 0), cv::Point (w-1, h-1), rcll_draw::getColor(rcll_draw::C_RED), 1);
    }

    rcll_draw::mergeImages(mat, origin, x, y, scale);
}
