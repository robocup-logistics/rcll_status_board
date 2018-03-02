#include <RobotMarker.h>

// RobotMarker ####################################################################
rcll_draw::RobotMarker::RobotMarker(Team team){
    blbl_id.setAlignment(rcll_draw::CenterCenter);

    if (team == rcll_draw::CYAN){
        crc_robot.setBackgroundColor(rcll_draw::C_CYAN_DARK);
    } else if(team == rcll_draw::MAGENTA){
        crc_robot.setBackgroundColor(rcll_draw::C_MAGENTA_DARK);
    } else {
        crc_robot.setBackgroundColor(rcll_draw::C_WHITE);
    }
    blbl_id.setBackgroundColor(rcll_draw::C_TRANSPARENT);

    crc_robot.setBorderColor(rcll_draw::C_BLACK);
    ln_direction.setBorderColor(rcll_draw::C_BLACK);
    blbl_id.setBorderColor(rcll_draw::C_TRANSPARENT);

    crc_robot.setBorderSize(2);
    ln_direction.setBorderSize(2);

    blbl_id.setFontSize(0.7);
    blbl_id.setFrontColor(rcll_draw::C_WHITE);
}

rcll_draw::RobotMarker::~RobotMarker(){

}

void rcll_draw::RobotMarker::setOrigin(int x0, int y0, int pixel_per_meter){
    this->x0 = x0;
    this->y0 = y0;
    this->pixel_per_meter = pixel_per_meter;
}

void rcll_draw::RobotMarker::setPos(double x, double y, double yaw){
    double ang = -(yaw+M_PI);
    int offset_x = x0 - x * pixel_per_meter;
    int offset_y = y0 + y * pixel_per_meter;
    int r1_x = diameter/4 * cos(ang+M_PI) * pixel_per_meter;
    int r1_y = diameter/4 * sin(ang+M_PI) * pixel_per_meter;
    int r2_x = 3*diameter/4 * cos(ang+M_PI) * pixel_per_meter;
    int r2_y = 3*diameter/4 * sin(ang+M_PI) * pixel_per_meter;
    crc_robot.setPos(offset_x, offset_y);
    crc_robot.setSize(pixel_per_meter * diameter/2);
    ln_direction.setLineByPoints(offset_x + r1_x, offset_y + r1_y, offset_x + r2_x, offset_y + r2_y);
    blbl_id.setPos(offset_x - pixel_per_meter * diameter / 2, offset_y - pixel_per_meter * diameter / 2);
    blbl_id.setSize(pixel_per_meter * diameter, pixel_per_meter * diameter);
}

void rcll_draw::RobotMarker::setRobotParams(std::string name_str, int id, double d){
    this->name = name_str;
    this->id = id;
    this->diameter = d;
    blbl_id.setContent(name_str);
}

void rcll_draw::RobotMarker::draw(cv::Mat &mat){
    crc_robot.draw(mat);
    ln_direction.draw(mat);
    blbl_id.draw(mat);
}
