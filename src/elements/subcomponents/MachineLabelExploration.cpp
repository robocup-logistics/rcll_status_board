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

#include <MachineLabelExploration.h>

// MachineLabelExploration ####################################################################

rcll_draw::MachineLabelExploration::MachineLabelExploration(){
    blbl_machinename.setAlignment(rcll_draw::CenterLeft);
    blbl_status1.setAlignment(rcll_draw::CenterCenter);
    blbl_status2.setAlignment(rcll_draw::CenterCenter);

    blbl_machinename.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_status1.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_status2.setBackgroundColor(rcll_draw::C_GREY_LIGHT);

    blbl_machinename.setBorderColor(rcll_draw::C_WHITE);
    blbl_status1.setBorderColor(rcll_draw::C_WHITE);
    blbl_status2.setBorderColor(rcll_draw::C_WHITE);

    blbl_machinename.setFontSize(0.9);
    blbl_status1.setFontSize(0.7);
    blbl_status2.setFontSize(0.7);

    blbl_machinename.setFrontColor(rcll_draw::C_WHITE);
    blbl_status1.setFrontColor(rcll_draw::C_BLACK);
    blbl_status2.setFrontColor(rcll_draw::C_BLACK);

    img_status1.setImage(rcll_draw::readImage(rcll_draw::getFile(4,4)));
    img_status2.setImage(rcll_draw::readImage(rcll_draw::getFile(4,4)));
}

rcll_draw::MachineLabelExploration::~MachineLabelExploration(){

}

void rcll_draw::MachineLabelExploration::setGeometry(int x, int y, int w, int h){


    if (!short_display){
        blbl_machinename.setSize(w * 0.34, h);
        blbl_status1.setSize(w * 0.33, h);
        blbl_status2.setSize(w * 0.33, h);
        img_status1.setScale(0.25);
        img_status2.setScale(0.25);

        blbl_machinename.setFontSize(0.9);

        blbl_machinename.setPos(x, y);
        blbl_status1.setPos(x + w * 0.34, y);
        blbl_status2.setPos(x + w * 0.67, y);

        img_status1.setPos(x + w * 0.57, y + (h - img_status1.getH())/2);
        img_status2.setPos(x + w * 0.9, y + (h - img_status2.getH())/2);
    } else {
        blbl_machinename.setSize(w * 0.5, h);
        blbl_status1.setSize(w * 0.25, h);
        blbl_status2.setSize(w * 0.25, h);
        img_status1.setScale(0.25);
        img_status2.setScale(0.25);

        blbl_machinename.setFontSize(1.0);

        blbl_machinename.setPos(x, y);
        blbl_status1.setPos(x + w * 0.5, y);
        blbl_status2.setPos(x + w * 0.75, y);

        img_status1.setPos(x + w * 0.5 + (w * 0.25 - img_status1.getW())/2, y + (h - img_status1.getH())/2);
        img_status2.setPos(x + w * 0.75 + (w * 0.25 - img_status2.getW())/2, y + (h - img_status2.getH())/2);
    }
}

void rcll_draw::MachineLabelExploration::setShortDisplay(bool short_display){
    this->short_display = short_display;
}

void rcll_draw::MachineLabelExploration::setMachine(rcll_vis_msgs::Machine &machine){
    blbl_machinename.setContent(" " + machine.name_long + " (" + machine.name_short + ")");
    blbl_status1.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_status2.setBackgroundColor(rcll_draw::C_GREY_LIGHT);

    std::string status1, status2;

    if (machine.machine_status_exploration1 == 0){
        status1 = "unreported";
        img_status1.setImage(rcll_draw::readImage(rcll_draw::getFile(4,4)));
    } else if (machine.machine_status_exploration1 == 1){
        status1 = "correct";
        img_status1.setImage(rcll_draw::readImage(rcll_draw::getFile(5,4)));
    } else if (machine.machine_status_exploration1 == 2){
        status1 = "wrong";
        img_status1.setImage(rcll_draw::readImage(rcll_draw::getFile(6,4)));
    } else {
        status1 = "unknown";
        img_status1.setImage(rcll_draw::readImage(""));
    }

    if (machine.machine_status_exploration2 == 0){
        status2 = "unreported";
        img_status2.setImage(rcll_draw::readImage(rcll_draw::getFile(4,4)));
    } else if (machine.machine_status_exploration2 == 1){
        status2 = "correct";
        img_status2.setImage(rcll_draw::readImage(rcll_draw::getFile(5,4)));
    } else if (machine.machine_status_exploration2 == 2){
        status2 = "wrong";
        img_status2.setImage(rcll_draw::readImage(rcll_draw::getFile(6,4)));
    } else {
        status2 = "unknown";
        img_status2.setImage(rcll_draw::readImage(""));
    }

    if (!short_display){
        blbl_status1.setContent(status1);
        blbl_status2.setContent(status2);
    }
}

void rcll_draw::MachineLabelExploration::setHeader(std::string status1, std::string status2){
    blbl_machinename.setContent("");
    blbl_status1.setContent(status1);
    blbl_status2.setContent(status2);

    blbl_machinename.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_status1.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_status2.setBackgroundColor(rcll_draw::C_BLACK);

    blbl_machinename.setFrontColor(rcll_draw::C_WHITE);
    blbl_status1.setFrontColor(rcll_draw::C_WHITE);
    blbl_status2.setFrontColor(rcll_draw::C_WHITE);

    blbl_machinename.setFontSize(1.0);
    blbl_status1.setFontSize(1.0);
    blbl_status2.setFontSize(1.0);

    img_status1.setImage(rcll_draw::readImage(""));
    img_status2.setImage(rcll_draw::readImage(""));
}

void rcll_draw::MachineLabelExploration::draw(cv::Mat &mat){
    blbl_machinename.draw(mat);

    blbl_status1.draw(mat);
    blbl_status2.draw(mat);

    img_status1.draw(mat, rcll_draw::getColor(rcll_draw::C_WHITE));
    img_status2.draw(mat, rcll_draw::getColor(rcll_draw::C_WHITE));
}
