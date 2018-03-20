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

void rcll_draw::MachineInfoProduction::setMachineName(std::string name, int index){
    if (index >= 0 && index < 7){
        machines[index].setMachineName(" " + name);
    }
}

void rcll_draw::MachineInfoProduction::setMachineStatus(std::string status, int index){
    if (index >= 0 && index < 7){
        Color lamp1, lamp2;
        std::string lamp1_str, lamp2_str, status_str = "";
        bool flashing = false;
        if (status == "Idle"){
            lamp1 = rcll_draw::C_GREEN_LIGHT;
            lamp2 = rcll_draw::C_GREEN_LIGHT;
            lamp1_str = "Free For";
            lamp2_str = "Production";
        } else if (status == "Broken"){
            lamp1 = rcll_draw::C_RED;
            lamp2 = rcll_draw::C_YELLOW;
            flashing = true;
            lamp1_str = "Incorrect";
            lamp2_str = "Instruction";
        } else if (status == "Processing"){
            lamp1 = rcll_draw::C_GREEN_LIGHT;
            lamp2 = rcll_draw::C_YELLOW;
            lamp1_str = "Processing";
            lamp2_str = "Product";
        } else if (status == "Prepared"){
            lamp1 = rcll_draw::C_GREEN_LIGHT;
            lamp2 = rcll_draw::C_GREEN_LIGHT;
            lamp1_str = "Prepared";
            lamp2_str = "For Product";
            flashing = true;
        } else if (status == "Down"){
            lamp1 = rcll_draw::C_RED;
            lamp2 = rcll_draw::C_RED;
            lamp1_str = "Scheduled";
            lamp2_str = "Down";
        } else if (status == "Finished"){
            lamp1 = rcll_draw::C_YELLOW;
            lamp2 = rcll_draw::C_YELLOW;
            lamp1_str = "Finished";
            lamp2_str = "Product";
        } else if (status == "Waiting"){
            lamp1 = rcll_draw::C_YELLOW;
            lamp2 = rcll_draw::C_YELLOW;
            flashing = true;
            lamp1_str = "Waiting For";
            lamp2_str = "Removed";
        } else if (status == "Offline"){
            lamp1 = rcll_draw::C_GREY_LIGHT;
            lamp2 = rcll_draw::C_GREY_LIGHT;
            status_str = "Offline";
        } else {
            lamp1 = rcll_draw::C_GREY_LIGHT;
            lamp2 = rcll_draw::C_GREY_LIGHT;
            status_str = "";
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
