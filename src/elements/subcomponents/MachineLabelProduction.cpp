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

#include <MachineLabelProduction.h>

// MachineLabelProduction ####################################################################

rcll_draw::MachineLabelProduction::MachineLabelProduction(rcll_draw::Team team){
    blbl_machinename.setAlignment(rcll_draw::CenterLeft);
    blbl_lamp1.setAlignment(rcll_draw::CenterCenter);
    blbl_lamp2.setAlignment(rcll_draw::CenterCenter);

    if (team == rcll_draw::CYAN){
        blbl_machinename.setBackgroundColor(rcll_draw::C_BLACK);
        blbl_machinename.setFrontColor(rcll_draw::C_WHITE);
    } else if (team == rcll_draw::MAGENTA){
        blbl_machinename.setBackgroundColor(rcll_draw::C_BLACK);
        blbl_machinename.setFrontColor(rcll_draw::C_WHITE);
    } else {
        blbl_machinename.setBackgroundColor(rcll_draw::C_BLACK);
        blbl_machinename.setFrontColor(rcll_draw::C_WHITE);
    }
    blbl_lamp1.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_lamp2.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_border.setBackgroundColor(rcll_draw::C_TRANSPARENT);

    blbl_lamp1.setFrontColor(rcll_draw::C_BLACK);
    blbl_lamp2.setFrontColor(rcll_draw::C_BLACK);

    blbl_machinename.setBorderColor(rcll_draw::C_TRANSPARENT);
    blbl_lamp1.setBorderColor(rcll_draw::C_WHITE);
    blbl_lamp2.setBorderColor(rcll_draw::C_WHITE);
    blbl_border.setBorderColor(rcll_draw::C_WHITE);

    blbl_machinename.setFontSize(0.9);
    blbl_lamp1.setFontSize(0.7);
    blbl_lamp2.setFontSize(0.7);



    flashing = false;
}

rcll_draw::MachineLabelProduction::~MachineLabelProduction(){

}

void rcll_draw::MachineLabelProduction::setGeometry(int x, int y, int w, int h){
    blbl_machinename.setPos(x, y);
    blbl_lamp1.setPos(x + w * 0.7, y);
    blbl_lamp2.setPos(x + w * 0.7, y + h/2);
    blbl_border.setPos(x, y);

    blbl_machinename.setSize(w * 0.7, h);
    blbl_lamp1.setSize(w * 0.3, h/2);
    blbl_lamp2.setSize(w * 0.3, h/2);
    blbl_border.setSize(w, h);
}

void rcll_draw::MachineLabelProduction::setMachine(rcll_vis_msgs::Machine &machine){
    blbl_machinename.setContent(" " + machine.name_long + " (" + machine.name_short + ")");

    flashing = false;
    if (machine.machine_status_production == "IDLE"){
        lamp1_color = rcll_draw::C_GREEN_LIGHT;
        lamp2_color = rcll_draw::C_GREEN_LIGHT;
        blbl_lamp1.setContent("Free For");
        blbl_lamp2.setContent("Production");
    } else if (machine.machine_status_production == "BROKEN"){
        lamp1_color = rcll_draw::C_RED;
        lamp2_color = rcll_draw::C_YELLOW;
        flashing = true;
        blbl_lamp1.setContent("Incorrect");
        blbl_lamp2.setContent("Instruction");
    } else if (machine.machine_status_production == "PROCESSING" || machine.machine_status_production == "PROCESSED"){
        lamp1_color = rcll_draw::C_GREEN_LIGHT;
        lamp2_color = rcll_draw::C_YELLOW;
        blbl_lamp1.setContent("Processing");
        blbl_lamp2.setContent("Product");
    } else if (machine.machine_status_production == "PREPARED"){
        lamp1_color = rcll_draw::C_GREEN_LIGHT;
        lamp2_color = rcll_draw::C_GREEN_LIGHT;
        blbl_lamp1.setContent("Prepared");
        blbl_lamp2.setContent("For Product");
        flashing = true;
    } else if (machine.machine_status_production == "DOWN"){
        lamp1_color = rcll_draw::C_RED;
        lamp2_color = rcll_draw::C_RED;
        blbl_lamp1.setContent("Scheduled");
        blbl_lamp2.setContent("Down");
    } else if (machine.machine_status_production == "READY-AT-OUTPUT"){
        lamp1_color = rcll_draw::C_YELLOW;
        lamp2_color = rcll_draw::C_YELLOW;
        blbl_lamp1.setContent("Finished");
        blbl_lamp2.setContent("Product");
    } else if (machine.machine_status_production == "WAIT-IDLE"){
        lamp1_color = rcll_draw::C_YELLOW;
        lamp2_color = rcll_draw::C_YELLOW;
        flashing = true;
        blbl_lamp1.setContent("Waiting For");
        blbl_lamp2.setContent("Removed");
    } else if (machine.machine_status_production == "OFFLINE"){
        lamp1_color = rcll_draw::C_GREY_LIGHT;
        lamp2_color = rcll_draw::C_GREY_LIGHT;
        blbl_lamp1.setContent("Machine");
        blbl_lamp2.setContent("Offline");
    } else {
        lamp1_color = rcll_draw::C_GREY_LIGHT;
        lamp2_color = rcll_draw::C_GREY_LIGHT;
        blbl_lamp1.setContent("Machine");
        blbl_lamp2.setContent("Offline");
    }

    blbl_lamp1.setBackgroundColor(lamp1_color);
    blbl_lamp2.setBackgroundColor(lamp2_color);
}

void rcll_draw::MachineLabelProduction::draw(cv::Mat &mat){
    if (flashing){
        if (getBoolSignal(ros::Time::now(), ros::Rate(1.33))){
            blbl_lamp1.setBackgroundColor(lamp1_color);
            blbl_lamp2.setBackgroundColor(lamp2_color);
            blbl_lamp1.setBorderColor(lamp1_color);
            blbl_lamp2.setBorderColor(lamp2_color);
        } else {
            blbl_lamp1.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
            blbl_lamp2.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
            blbl_lamp1.setBorderColor(rcll_draw::C_GREY_LIGHT);
            blbl_lamp2.setBorderColor(rcll_draw::C_GREY_LIGHT);
        }
    } else {
        blbl_lamp1.setBackgroundColor(lamp1_color);
        blbl_lamp2.setBackgroundColor(lamp2_color);
        blbl_lamp1.setBorderColor(lamp1_color);
        blbl_lamp2.setBorderColor(lamp2_color);
    }

    blbl_machinename.draw(mat);
    blbl_lamp1.draw(mat);
    blbl_lamp2.draw(mat);
    blbl_border.draw(mat);
}
