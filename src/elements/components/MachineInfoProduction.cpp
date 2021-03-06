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

#include <MachineInfoProduction.h>

// MachineInfoProduction ####################################################################

rcll_draw::MachineInfoProduction::MachineInfoProduction() : rcll_draw::MachineInfoProduction::MachineInfoProduction(rcll_draw::NO_TEAM){

}

rcll_draw::MachineInfoProduction::MachineInfoProduction(Team team){
    origin = cv::Mat(h, w, CV_8UC4);
    blbl_header.setAlignment(rcll_draw::CenterCenter);
    blbl_header.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_header.setFontSize(2.0);
    blbl_header.setContent("MACHINES");
    if (team == rcll_draw::CYAN){
        blbl_header.setFrontColor(rcll_draw::C_CYAN_DARK);
    } else if (team == rcll_draw::MAGENTA){
        blbl_header.setFrontColor(rcll_draw::C_MAGENTA_DARK);
    } else {
        blbl_header.setFrontColor(rcll_draw::C_BLACK);
    }

    for (size_t i = 0; i < keys.size(); i++){
        mlp_machines.push_back(MachineLabelProduction(team));
        machine_map[keys[i]] = i;
    }
    this->team = team;
}

rcll_draw::MachineInfoProduction::~MachineInfoProduction(){

}

void rcll_draw::MachineInfoProduction::setGeometry(int x, int y, double scale){
    this->x = x;
    this->y = y;
    this->scale = scale;

    if (!short_display){
        blbl_header.setPos(0, 0);
        blbl_header.setSize(w, h * 0.2);

        int w1 = (w - gapsize) / 2;

        for (size_t i = 0; i < keys.size(); i++){
            if (i < 3){
                mlp_machines[machine_map[keys[i]]].setGeometry(0, (i+1) * h * 0.2, w1, h * 0.2);
            } else {
                mlp_machines[machine_map[keys[i]]].setGeometry(w1 + gapsize, (i-2) * h * 0.2, w1, h*0.2);
            }
        }
    } else {
        blbl_header.setPos(0, 0);
        blbl_header.setSize(w, h / 8);

        for (size_t i = 0; i < keys.size(); i++){
            mlp_machines[machine_map[keys[i]]].setGeometry(0, (i+1) * h / 8, w, h / 8);
        }
    }
}

void rcll_draw::MachineInfoProduction::setShortDisplay(bool short_display){
    this->short_display = short_display;

    if (short_display){
        w = 480;
        h = 552;
        origin = cv::Mat(h, w, CV_8UC4);
    }
}

int rcll_draw::MachineInfoProduction::getW(double scale){
    return (int)((double)w * scale);
}

int rcll_draw::MachineInfoProduction::getH(double scale){
    return (int)((double)h * scale);
}

void rcll_draw::MachineInfoProduction::setMachines(std::vector<rcll_vis_msgs::Machine> &machines){
    for (size_t i = 0; i < machines.size(); i++){
        if (team == (rcll_draw::Team)machines[i].team){
            mlp_machines[machine_map[machines[i].key]].setMachine(machines[i]);

        }
    }
}

void rcll_draw::MachineInfoProduction::draw(cv::Mat &mat, bool show_element_border){
    cv::rectangle(origin, cv::Point(0, 0), cv::Point (w-1, h-1), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED);
    blbl_header.draw(origin);
    for (size_t i = 0; i < keys.size(); i++){
        mlp_machines[machine_map[keys[i]]].draw(origin);
    }
    if (show_element_border){
        cv::rectangle(origin, cv::Point(0, 0), cv::Point (w-1, h-1), rcll_draw::getColor(rcll_draw::C_RED), 1);
    }
    rcll_draw::mergeImages(mat, origin, x, y, scale);
}
