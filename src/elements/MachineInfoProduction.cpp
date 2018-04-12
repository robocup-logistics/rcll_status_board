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

    machines.resize(7);

    for (size_t i = 0; i < machines.size(); i++){
        machines[i].setFlashing(false);
        machines[i].setMachineStatus("Offline", rcll_draw::C_GREY_LIGHT, rcll_draw::C_GREY_LIGHT, "", "");
    }
}

rcll_draw::MachineInfoProduction::~MachineInfoProduction(){

}

void rcll_draw::MachineInfoProduction::setGeometry(int x, int y, int w, int h, int gapsize){
    int w1 = (w - gapsize) / 2;
    blbl_header.setPos(x, y);
    blbl_header.setSize(w, h*0.2);
    machines[0].setGeometry(x, y + 1*h*0.2, w1, h*0.2);
    machines[1].setGeometry(x, y + 2*h*0.2, w1, h*0.2);
    machines[2].setGeometry(x, y + 3*h*0.2, w1, h*0.2);
    machines[3].setGeometry(x+w1+gapsize, y + 1*h*0.2, w1, h*0.2);
    machines[4].setGeometry(x+w1+gapsize, y + 2*h*0.2, w1, h*0.2);
    machines[5].setGeometry(x+w1+gapsize, y + 3*h*0.2, w1, h*0.2);
    machines[6].setGeometry(x+w1+gapsize, y + 4*h*0.2, w1, h*0.2);
}

void rcll_draw::MachineInfoProduction::setMachineName(std::string name_long, std::string name_short, int index){
    if (index >= 0 && index < 7){
        machines[index].setMachineName(" " + name_long + " (" + name_short + ")");
    }
}

void rcll_draw::MachineInfoProduction::setMachineStatus(std::string status, int index){
    if (index >= 0 && index < 7){
        Color lamp1, lamp2;
        std::string lamp1_str, lamp2_str, status_str = "";
        bool flashing = false;
        if (status == "IDLE"){
            lamp1 = rcll_draw::C_GREEN_LIGHT;
            lamp2 = rcll_draw::C_GREEN_LIGHT;
            lamp1_str = "Free For";
            lamp2_str = "Production";
        } else if (status == "BROKEN"){
            lamp1 = rcll_draw::C_RED;
            lamp2 = rcll_draw::C_YELLOW;
            flashing = true;
            lamp1_str = "Incorrect";
            lamp2_str = "Instruction";
        } else if (status == "PROCESSING" || status == "PROCESSED"){
            lamp1 = rcll_draw::C_GREEN_LIGHT;
            lamp2 = rcll_draw::C_YELLOW;
            lamp1_str = "Processing";
            lamp2_str = "Product";
        } else if (status == "PREPARED"){
            lamp1 = rcll_draw::C_GREEN_LIGHT;
            lamp2 = rcll_draw::C_GREEN_LIGHT;
            lamp1_str = "Prepared";
            lamp2_str = "For Product";
            flashing = true;
        } else if (status == "DOWN"){
            lamp1 = rcll_draw::C_RED;
            lamp2 = rcll_draw::C_RED;
            lamp1_str = "Scheduled";
            lamp2_str = "Down";
        } else if (status == "READY-AT-OUTPUT"){
            lamp1 = rcll_draw::C_YELLOW;
            lamp2 = rcll_draw::C_YELLOW;
            lamp1_str = "Finished";
            lamp2_str = "Product";
        } else if (status == "WAIT-IDLE"){
            lamp1 = rcll_draw::C_YELLOW;
            lamp2 = rcll_draw::C_YELLOW;
            flashing = true;
            lamp1_str = "Waiting For";
            lamp2_str = "Removed";
        } else if (status == "OFFLINE"){
            lamp1 = rcll_draw::C_GREY_LIGHT;
            lamp2 = rcll_draw::C_GREY_LIGHT;
            status_str = "Offline";
        } else {
            lamp1 = rcll_draw::C_GREY_LIGHT;
            lamp2 = rcll_draw::C_GREY_LIGHT;
            status_str = "Offline";
        }
        machines[index].setFlashing(flashing);
        machines[index].setMachineStatus(status_str, lamp1, lamp2, lamp1_str, lamp2_str);
    }
}

void rcll_draw::MachineInfoProduction::draw(cv::Mat &mat){
    blbl_header.draw(mat);
    machines[0].draw(mat);
    machines[1].draw(mat);
    machines[2].draw(mat);
    machines[3].draw(mat);
    machines[4].draw(mat);
    machines[5].draw(mat);
    machines[6].draw(mat);
}
