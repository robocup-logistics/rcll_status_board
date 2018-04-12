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

    machines.resize(8);
    machines[0].setHeader("Position", "Orientation");
}

rcll_draw::MachineInfoExploration::~MachineInfoExploration(){

}

void rcll_draw::MachineInfoExploration::setGeometry(int x, int y, int w, int h, int gapsize){
    blbl_header1.setPos(x, y);
    blbl_header2.setPos(x, y + h/11);
    blbl_header1.setSize(w, h/11);
    blbl_header2.setSize(w, h/11);
    machines[0].setGeometry(x, y + 3*h/11, w, h/11);
    machines[1].setGeometry(x, y + 4*h/11, w, h/11);
    machines[2].setGeometry(x, y + 5*h/11, w, h/11);
    machines[3].setGeometry(x, y + 6*h/11, w, h/11);
    machines[4].setGeometry(x, y + 7*h/11, w, h/11);
    machines[5].setGeometry(x, y + 8*h/11, w, h/11);
    machines[6].setGeometry(x, y + 9*h/11, w, h/11);
    machines[7].setGeometry(x, y + 10*h/11, w, h/11);
}

void rcll_draw::MachineInfoExploration::setMachineName(std::string name_long, std::string name_short, int index){
    if (index >= 0 && index < 7){
        machines[index + 1].setMachineName(" " + name_long + " (" + name_short + ")", index + 1);
    }
}

void rcll_draw::MachineInfoExploration::setMachineStatus(int status1, int status2, int index){
    if (index >= 0 && index < 7){
        machines[index + 1].setMachineStatus(status1, status2);
    }
}

void rcll_draw::MachineInfoExploration::draw(cv::Mat &mat){
    blbl_header1.draw(mat);
    blbl_header2.draw(mat);
    machines[0].draw(mat);
    machines[1].draw(mat);
    machines[2].draw(mat);
    machines[3].draw(mat);
    machines[4].draw(mat);
    machines[5].draw(mat);
    machines[6].draw(mat);
    machines[7].draw(mat);
}
