#include <MachineMarker.h>

// MachineMarker ####################################################################
rcll_draw::MachineMarker::MachineMarker(){
    team = rcll_draw::NO_TEAM;
}

rcll_draw::MachineMarker::MachineMarker(rcll_draw::Team team_){
    img = cv::Mat(0, 0, CV_8UC4);
    blbl_machine.setAlignment(rcll_draw::CenterCenter);
    blbl_in.setAlignment(rcll_draw::CenterCenter);
    blbl_out.setAlignment(rcll_draw::CenterCenter);

    if (team_ == rcll_draw::CYAN){
        blbl_machine.setBackgroundColor(rcll_draw::C_CYAN_LIGHT);
    } else if(team_ == rcll_draw::MAGENTA){
        blbl_machine.setBackgroundColor(rcll_draw::C_MAGENTA_LIGHT);
    } else {
        blbl_machine.setBackgroundColor(rcll_draw::C_WHITE);
    }
    team = team_;

    blbl_in.setBackgroundColor(rcll_draw::C_TRANSPARENT);
    blbl_out.setBackgroundColor(rcll_draw::C_TRANSPARENT);

    blbl_in.setBorderColor(rcll_draw::C_TRANSPARENT);
    blbl_out.setBorderColor(rcll_draw::C_TRANSPARENT);
    blbl_machine.setBorderColor(rcll_draw::C_BLACK);

    blbl_machine.setBorderSize(2);
    blbl_in.setBorderSize(2);
    blbl_out.setBorderSize(2);

    blbl_machine.setFontSize(0.8);
    blbl_in.setFontSize(0.5);
    blbl_out.setFontSize(0.5);

    blbl_machine.setFrontColor(rcll_draw::C_BLACK);
    blbl_in.setFrontColor(rcll_draw::C_GREEN_DARK);
    blbl_out.setFrontColor(rcll_draw::C_RED);

    ln_input.setBorderColor(rcll_draw::C_BLACK);
    crc_output.setBorderColor(rcll_draw::C_BLACK);
    crc_output.setBackgroundColor(rcll_draw::C_BLACK);
    ln_input.setBorderSize(2);
    crc_output.setBorderSize(2);
}

rcll_draw::MachineMarker::~MachineMarker(){

}

void rcll_draw::MachineMarker::setPhase(rcll_draw::GamePhase gamephase){
    this->gamephase = gamephase;
    recalculate();
}

void rcll_draw::MachineMarker::setOrigin(int x0, int y0, int pixel_per_meter){
    this->x0 = x0;
    this->y0 = y0;
    this->pixel_per_meter = pixel_per_meter;
}

void rcll_draw::MachineMarker::setPos(double x, double y, double yaw){
    this->yaw = std::fmod(yaw + M_PI / 2, 2 * M_PI);
    xm = x0 - x * pixel_per_meter;
    ym = y0 + y * pixel_per_meter;

    recalculate();
}


void rcll_draw::MachineMarker::setMachineParams(std::string name, double w, double h){
    this->w = w;
    this->h = h;
    blbl_machine.setContent(name);
    blbl_in.setContent("IN");
    blbl_out.setContent("OUT");
    blbl_machine.setSize(w * pixel_per_meter, h * pixel_per_meter);
    blbl_in.setSize(w * pixel_per_meter, h * pixel_per_meter);
    blbl_out.setSize(w * pixel_per_meter, h * pixel_per_meter);
}

void rcll_draw::MachineMarker::setExplorationIcons(int left, int right){
    this->exploration_icon_left = left;
    this->exploration_icon_right = right;
}

rcll_draw::Team rcll_draw::MachineMarker::getTeam(){
    return team;
}

void rcll_draw::MachineMarker::recalculate(){
    double ang;
    if (yaw > M_PI / 2 && yaw <= 3 * M_PI / 2){
        ang = yaw + M_PI;
    } else {
        ang = yaw;
    }

    // calculate the unrotated machine image
    cv::Mat tmp = cv::Mat(w * pixel_per_meter * 2.0, w * pixel_per_meter * 2.0, CV_8UC4);
    cv::rectangle(tmp, cv::Point(0,0), cv::Point(tmp.cols, tmp.rows), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED, 8, 0);
    blbl_machine.setPos((tmp.cols - w * pixel_per_meter) / 2, (tmp.rows - h * pixel_per_meter ) / 2);
    if (yaw > M_PI / 2 && yaw <= 3 * M_PI / 2){
        blbl_in.setPos((tmp.cols - w * pixel_per_meter) / 2, (tmp.rows - h * pixel_per_meter) / 2 + h * pixel_per_meter * 0.75);
        blbl_out.setPos((tmp.cols - w * pixel_per_meter) / 2, (tmp.rows - h * pixel_per_meter) / 2 - h * pixel_per_meter * 0.8);
    } else {
        blbl_in.setPos((tmp.cols - w * pixel_per_meter) / 2, (tmp.rows - h * pixel_per_meter) / 2 - h * pixel_per_meter * 0.8);
        blbl_out.setPos((tmp.cols - w * pixel_per_meter) / 2, (tmp.rows - h * pixel_per_meter) / 2 + h * pixel_per_meter * 0.75);
    }
    blbl_machine.draw(tmp);

    if (gamephase == rcll_draw::SETUP){
        blbl_in.draw(tmp);
        blbl_out.draw(tmp);
    } else if (gamephase == rcll_draw::EXPLORATION){
        cv::Mat left = rcll_draw::readImage(rcll_draw::getFile(exploration_icon_left, 5));
        cv::resize(left, left, cv::Size(), 0.15, 0.15, cv::INTER_NEAREST);
        rcll_draw::mergeImages(tmp, left, rcll_draw::getColor(rcll_draw::C_WHITE), 0.25 * tmp.cols, 0.9 * tmp.rows - left.rows);

        cv::Mat right = rcll_draw::readImage(rcll_draw::getFile(exploration_icon_right, 5));
        cv::resize(right, right, cv::Size(), 0.15, 0.15, cv::INTER_NEAREST);
        rcll_draw::mergeImages(tmp, right, rcll_draw::getColor(rcll_draw::C_WHITE), 0.5 * tmp.cols, 0.9 * tmp.rows - right.rows);
    }

    // calcualte the rotated machine image
    cv::Point2f src_center(tmp.cols / 2.0F, tmp.rows / 2.0F);
    cv::Mat rot_mat = cv::getRotationMatrix2D(src_center, ang * 180.0 / M_PI, 1.0);
    cv::warpAffine(tmp, img, rot_mat, tmp.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, rcll_draw::getColor(rcll_draw::C_WHITE));
}

void rcll_draw::MachineMarker::draw(cv::Mat &mat){
    rcll_draw::mergeImages(mat, img, rcll_draw::getColor(rcll_draw::C_WHITE), xm - img.cols / 2, ym - img.rows / 2);
}
