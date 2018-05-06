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

#include <HeaderPanel.h>

// HeaderPanel ####################################################################
rcll_draw::HeaderPanel::HeaderPanel(std::string content, rcll_draw::Team team){
    blbl_header.setContent(content);
    blbl_header.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_header.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_header.setBorderColor(rcll_draw::C_WHITE);
    if (team == rcll_draw::CYAN){
        blbl_header.setFrontColor(rcll_draw::C_CYAN_DARK);
    } else if (team == rcll_draw::MAGENTA){
        blbl_header.setFrontColor(rcll_draw::C_MAGENTA_DARK);
    } else {
        blbl_header.setFrontColor(rcll_draw::C_BLACK);
    }
}

rcll_draw::HeaderPanel::~HeaderPanel(){

}

void rcll_draw::HeaderPanel::setGeometry(int y, int w, int h){
    blbl_header.setPos(0, y);
    blbl_header.setSize(w, h);
    blbl_header.setFontSize(2.0);

    ROS_INFO("HeaderPanel w=%i h=%i", w, h);
}

void rcll_draw::HeaderPanel::draw(cv::Mat &mat){
    blbl_header.draw(mat);
}
