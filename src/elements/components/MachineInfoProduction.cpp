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

rcll_draw::MachineInfoProduction::MachineInfoProduction(){

}

rcll_draw::MachineInfoProduction::MachineInfoProduction(Team team){
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
        mlp_machines.push_back(MachineLabelProduction());
        machine_map[keys[i]] = i;
    }
    this->team = team;
}

rcll_draw::MachineInfoProduction::~MachineInfoProduction(){

}

void rcll_draw::MachineInfoProduction::setGeometry(int x, int y, int w, int h, int gapsize){
    int w1 = (w - gapsize) / 2;
    blbl_header.setPos(x, y);
    blbl_header.setSize(w, h*0.2);

    for (size_t i = 0; i < keys.size(); i++){
        if (i < 4){
            mlp_machines[machine_map[keys[i]]].setGeometry(x, y + (i+1) * h * 0.2, w1, h * 0.2);
        } else {
            mlp_machines[machine_map[keys[i]]].setGeometry(x + w1 + gapsize, y + (i-3) * h * 0.2, w1, h*0.2);
        }
    }

    ROS_INFO("MachineInfoProduction w=%i h=%i", w, h);
}

void rcll_draw::MachineInfoProduction::setMachines(std::vector<rcll_vis_msgs::Machine> &machines){
    for (size_t i = 0; i < machines.size(); i++){
        if (team == (rcll_draw::Team)machines[i].team){
            mlp_machines[machine_map[machines[i].key]].setMachine(machines[i]);

        }
    }
}

void rcll_draw::MachineInfoProduction::draw(cv::Mat &mat){
    blbl_header.draw(mat);
    for (size_t i = 0; i < keys.size(); i++){
        mlp_machines[machine_map[keys[i]]].draw(mat);
    }
}
