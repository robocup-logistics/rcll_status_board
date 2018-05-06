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

#include <MachineInfoExploration.h>

// MachineInfoExploration ####################################################################
rcll_draw::MachineInfoExploration::MachineInfoExploration(){

}

rcll_draw::MachineInfoExploration::MachineInfoExploration(Team team){
    this->team = team;
    blbl_header1.setAlignment(rcll_draw::CenterCenter);
    blbl_header2.setAlignment(rcll_draw::CenterCenter);
    blbl_header1.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_header2.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_header1.setBorderColor(rcll_draw::C_WHITE);
    blbl_header2.setBorderColor(rcll_draw::C_WHITE);
    blbl_header1.setFontSize(2.0);
    blbl_header2.setFontSize(2.0);
    blbl_header1.setContent("MACHINE DETECTION");
    blbl_header2.setContent("REPORTS");
    if (team == rcll_draw::CYAN){
        blbl_header1.setFrontColor(rcll_draw::C_CYAN_DARK);
        blbl_header2.setFrontColor(rcll_draw::C_CYAN_DARK);
    } else if (team == rcll_draw::MAGENTA){
        blbl_header1.setFrontColor(rcll_draw::C_MAGENTA_DARK);
        blbl_header2.setFrontColor(rcll_draw::C_MAGENTA_DARK);
    } else {
        blbl_header1.setFrontColor(rcll_draw::C_BLACK);
        blbl_header2.setFrontColor(rcll_draw::C_BLACK);
    }

    for (size_t i = 0; i < keys.size(); i++){
        mle_machines.push_back(MachineLabelExploration());
        machine_map[keys[i]] = i;
    }
    mle_machines[machine_map["H"]].setHeader("Position", "Orientation");
}

rcll_draw::MachineInfoExploration::~MachineInfoExploration(){

}

void rcll_draw::MachineInfoExploration::setGeometry(int x, int y, int w, int h, int gapsize){
    blbl_header1.setPos(x, y);
    blbl_header2.setPos(x, y + h/11);
    blbl_header1.setSize(w, h/11);
    blbl_header2.setSize(w, h/11);
    for (size_t i = 0; i < keys.size(); i++){
        mle_machines[machine_map[keys[i]]].setGeometry(x, y + (i + 3)*h/11, w, h/11);
    }

    ROS_INFO("MachineInfoExploration w=%i h=%i", w, h);
}

void rcll_draw::MachineInfoExploration::setMachines(std::vector<rcll_vis_msgs::Machine> &machines){
    for (size_t i = 0; i < machines.size(); i++){
        if (team == (rcll_draw::Team)machines[i].team){
            mle_machines[machine_map[machines[i].key]].setMachine(machines[i]);
        }
    }
}

void rcll_draw::MachineInfoExploration::draw(cv::Mat &mat){
    blbl_header1.draw(mat);
    blbl_header2.draw(mat);
    for (size_t i = 0; i < keys.size(); i++){
        mle_machines[machine_map[keys[i]]].draw(mat);
    }
}
