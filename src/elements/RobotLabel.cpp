#include <RobotLabel.h>

// RobotLabel ####################################################################
rcll_draw::RobotLabel::RobotLabel(){
    blbl_name.setAlignment(rcll_draw::CenterLeft);
    mlblbl_activity.setAlignment(rcll_draw::TopLeft);
    blbl_activetime.setAlignment(rcll_draw::CenterLeft);
    blbl_maintenance.setAlignment(rcll_draw::CenterLeft);

    blbl_name.setBackgroundColor(rcll_draw::C_BLACK);
    mlblbl_activity.setBackgroundColor(rcll_draw::C_GREY_DARK);
    blbl_activetime.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_maintenance.setBackgroundColor(rcll_draw::C_GREY_DARK);

    blbl_name.setBorderColor(rcll_draw::C_WHITE);
    mlblbl_activity.setBorderColor(rcll_draw::C_WHITE);
    blbl_activetime.setBorderColor(rcll_draw::C_WHITE);
    blbl_maintenance.setBorderColor(rcll_draw::C_WHITE);

    blbl_name.setFontSize(0.9);
    mlblbl_activity.setFontSize(0.7);
    blbl_activetime.setFontSize(0.7);
    blbl_maintenance.setFontSize(0.7);

    blbl_name.setFrontColor(rcll_draw::C_WHITE);
    mlblbl_activity.setFrontColor(rcll_draw::C_BLACK);
    blbl_activetime.setFrontColor(rcll_draw::C_BLACK);
    blbl_maintenance.setFrontColor(rcll_draw::C_BLACK);

    mlblbl_activity.setContent("Activity: unknown");
    blbl_activetime.setContent(" Active Time: 0%");
    blbl_maintenance.setContent(" Maintenance: 0 / ?");
}

rcll_draw::RobotLabel::~RobotLabel(){

}

void rcll_draw::RobotLabel::setGeometry(int x, int y, int w, int h){
    blbl_name.setPos(x, y);
    mlblbl_activity.setPos(x, y + h/5);
    blbl_activetime.setPos(x, y + 3*h/5);
    blbl_maintenance.setPos(x, y + 4*h/5);

    blbl_name.setSize(w, h/5);
    mlblbl_activity.setSize(w, 2*h/5);
    blbl_activetime.setSize(w, h/5);
    blbl_maintenance.setSize(w, h/5);
}

void rcll_draw::RobotLabel::setRobotName(std::string name_str, bool active){
    blbl_name.setContent(" " + name_str);
    if (active){
        blbl_name.setFrontColor(rcll_draw::C_GREEN_LIGHT);
    } else {
        blbl_name.setFrontColor(rcll_draw::C_RED);
    }
}

void rcll_draw::RobotLabel::setRobotStatus(std::string activity_str, double active_time, int maintenance_count, int maintenance_max){
    mlblbl_activity.setContent("Activity: " + activity_str);
    blbl_activetime.setContent(" Active Time: " + std::to_string((int)(active_time * 100)) + "%");
    blbl_maintenance.setContent(" Maintenance: " + std::to_string(maintenance_count) + " / " + std::to_string(maintenance_max));
}

void rcll_draw::RobotLabel::draw(cv::Mat &mat){
    blbl_name.draw(mat);
    mlblbl_activity.draw(mat);
    blbl_activetime.draw(mat);
    blbl_maintenance.draw(mat);
}
