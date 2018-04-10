#include <RobotInfo.h>
// RobotInfo ####################################################################

rcll_draw::RobotInfo::RobotInfo(){

}

rcll_draw::RobotInfo::RobotInfo(Team team){
    blbl_header.setAlignment(rcll_draw::CenterCenter);
    blbl_header.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_header.setFontSize(2.0);
    blbl_header.setContent("ROBOTS");
    if (team == rcll_draw::CYAN){
        blbl_header.setFrontColor(rcll_draw::C_CYAN_DARK);
    } else if (team == rcll_draw::MAGENTA){
        blbl_header.setFrontColor(rcll_draw::C_MAGENTA_DARK);
    } else {
        blbl_header.setFrontColor(rcll_draw::C_BLACK);
    }

    robots.resize(3);
    robots[0].setRobotName(1, "Rob1", false);
    robots[1].setRobotName(2, "Rob2", false);
    robots[2].setRobotName(3, "Rob3", false);
}

rcll_draw::RobotInfo::~RobotInfo(){

}

void rcll_draw::RobotInfo::setGeometry(int x, int y, int w, int h, int gapsize){
    int w1 = (w - 2 * gapsize) / 3;
    blbl_header.setPos(x, y);
    blbl_header.setSize(w, h*0.2);
    robots[0].setGeometry(x, y + h*0.2, w1, h*0.8);
    robots[1].setGeometry(x + gapsize + w1, y + h*0.2, w1, h*0.8);
    robots[2].setGeometry(x + 2 * (gapsize + w1), y + h*0.2, w1, h*0.8);
}

void rcll_draw::RobotInfo::setRobotName(int id, std::string name, bool active, int index){
    if (index >= 0 && index < 3){
        robots[index].setRobotName(id, name, active);
    }
}

void rcll_draw::RobotInfo::setRobotStatus(std::string activity, double active_time, int maintenance, int maintenance_max, int index){
    if (index >= 0 && index < 3){
        robots[index].setRobotStatus(activity, active_time, maintenance, maintenance_max);
    }
}

void rcll_draw::RobotInfo::draw(cv::Mat &mat){
    blbl_header.draw(mat);
    for (size_t i = 0; i < robots.size(); i++){
        robots[i].draw(mat);
    }
}
