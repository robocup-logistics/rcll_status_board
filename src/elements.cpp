#ifndef RCLL_ELEMENTS_H
#define RCLL_ELEMENTS_H
#include <elements.h>

// HeaderPanel ####################################################################
rcll_draw::HeaderPanel::HeaderPanel(std::string content, rcll_draw::Team team){
    blbl_header.setContent(content);
    blbl_header.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_header.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_header.setBorderColor(rcll_draw::C_WHITE);
    if (team == rcll_draw::CYAN){
        blbl_header.setFrontColor(rcll_draw::C_CYAN_DARK);
    } else if (team == rcll_draw::MAGENTA){
        blbl_header.setFrontColor(rcll_draw::C_MAGENTA_DARK);
    } else {
        blbl_header.setFrontColor(rcll_draw::C_BLACK);
    }
}

rcll_draw::HeaderPanel::~HeaderPanel(){

}

void rcll_draw::HeaderPanel::setGeometry(int y, int w, int h){
    blbl_header.setPos(0, y);
    blbl_header.setSize(w, h);
    blbl_header.setFontSize(3.6);
}

void rcll_draw::HeaderPanel::draw(cv::Mat &mat){
    blbl_header.draw(mat);
}

// VStatusPanel ####################################################################
rcll_draw::VStatusPanel::VStatusPanel(){
    this->team = rcll_draw::NO_TEAM;
}


rcll_draw::VStatusPanel::VStatusPanel(rcll_draw::Team team){
    blbl_state_header.setContent("STATE");
    blbl_phase_header.setContent("PHASE");
    blbl_time_header.setContent("TIME");
    blbl_score_header.setContent("SCORE");

    blbl_state_header.setAlignment(rcll_draw::CenterCenter);
    blbl_phase_header.setAlignment(rcll_draw::CenterCenter);
    blbl_time_header.setAlignment(rcll_draw::CenterCenter);
    blbl_score_header.setAlignment(rcll_draw::CenterCenter);
    blbl_state_value.setAlignment(rcll_draw::CenterCenter);
    blbl_phase_value.setAlignment(rcll_draw::CenterCenter);
    blbl_time_value.setAlignment(rcll_draw::CenterCenter);
    blbl_score_value.setAlignment(rcll_draw::CenterCenter);

    blbl_state_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_phase_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_time_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_score_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_state_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_phase_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_time_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_score_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);

    blbl_state_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_phase_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_time_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_score_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_state_value.setBorderColor(rcll_draw::C_WHITE);
    blbl_phase_value.setBorderColor(rcll_draw::C_WHITE);
    blbl_time_value.setBorderColor(rcll_draw::C_WHITE);
    blbl_score_value.setBorderColor(rcll_draw::C_WHITE);

    blbl_state_header.setFontSize(0.9);
    blbl_phase_header.setFontSize(0.9);
    blbl_time_header.setFontSize(0.9);
    blbl_score_header.setFontSize(0.9);
    blbl_state_value.setFontSize(0.9);
    blbl_phase_value.setFontSize(0.9);
    blbl_time_value.setFontSize(0.9);
    blbl_score_value.setFontSize(2);

    blbl_state_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_phase_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_time_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_score_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_state_value.setFrontColor(rcll_draw::C_BLACK);
    blbl_phase_value.setFrontColor(rcll_draw::C_BLACK);
    blbl_time_value.setFrontColor(rcll_draw::C_BLACK);
    if (team == rcll_draw::CYAN){
        blbl_score_value.setFrontColor(rcll_draw::C_CYAN_DARK);
    } else if (team == rcll_draw::MAGENTA){
        blbl_score_value.setFrontColor(rcll_draw::C_MAGENTA_DARK);
    } else {
        blbl_score_value.setFrontColor(rcll_draw::C_BLACK);
    }
}

rcll_draw::VStatusPanel::~VStatusPanel(){

}

void rcll_draw::VStatusPanel::setGeometry(int x, int y, int w, int h){
    blbl_state_header.setPos(x, y);
    blbl_phase_header.setPos(x, y + 2 * h / 9);
    blbl_time_header.setPos(x, y + 4 * h / 9);
    blbl_score_header.setPos(x, y + 6 * h / 9);
    blbl_state_value.setPos(x, y + 1 * h / 9);
    blbl_phase_value.setPos(x, y + 3 * h / 9);
    blbl_time_value.setPos(x, y + 5 * h / 9);
    blbl_score_value.setPos(x, y + 7 * h / 9);

    blbl_state_header.setSize(w, h / 9);
    blbl_phase_header.setSize(w, h / 9);
    blbl_time_header.setSize(w, h / 9);
    blbl_score_header.setSize(w, h / 9);
    blbl_state_value.setSize(w, h / 9);
    blbl_phase_value.setSize(w, h / 9);
    blbl_time_value.setSize(w, h / 9);
    blbl_score_value.setSize(w, 2 * h / 9);
}

void rcll_draw::VStatusPanel::setContent(std::string gamestate, std::string gamephase, int time, int score){
    int min = time / 60;
    int sec = time % 60;
    std::string time_str = std::to_string(min) + "min " + std::to_string(sec) + "sec";
    blbl_state_value.setContent(gamestate);
    blbl_phase_value.setContent(gamephase);
    blbl_time_value.setContent(time_str);
    blbl_score_value.setContent(std::to_string(score));
}

void rcll_draw::VStatusPanel::draw(cv::Mat &mat){
    blbl_state_header.draw(mat);
    blbl_phase_header.draw(mat);
    blbl_time_header.draw(mat);
    blbl_score_header.draw(mat);
    blbl_state_value.draw(mat);
    blbl_phase_value.draw(mat);
    blbl_time_value.draw(mat);
    blbl_score_value.draw(mat);
}

// HStatusPanel ####################################################################
rcll_draw::HStatusPanel::HStatusPanel(){
    this->team = rcll_draw::NO_TEAM;
}


rcll_draw::HStatusPanel::HStatusPanel(rcll_draw::Team team){
    blbl_state_header.setContent("STATE");
    blbl_phase_header.setContent("PHASE");
    blbl_time_header.setContent("TIME");
    blbl_score_header.setContent("SCORE");

    blbl_state_header.setAlignment(rcll_draw::CenterCenter);
    blbl_phase_header.setAlignment(rcll_draw::CenterCenter);
    blbl_time_header.setAlignment(rcll_draw::CenterCenter);
    blbl_score_header.setAlignment(rcll_draw::CenterCenter);
    blbl_state_value.setAlignment(rcll_draw::CenterCenter);
    blbl_phase_value.setAlignment(rcll_draw::CenterCenter);
    blbl_time_value.setAlignment(rcll_draw::CenterCenter);
    blbl_score_value.setAlignment(rcll_draw::CenterCenter);

    blbl_state_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_phase_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_time_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_score_header.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_state_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_phase_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_time_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_score_value.setBackgroundColor(rcll_draw::C_GREY_LIGHT);

    blbl_state_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_phase_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_time_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_score_header.setBorderColor(rcll_draw::C_WHITE);
    blbl_state_value.setBorderColor(rcll_draw::C_WHITE);
    blbl_phase_value.setBorderColor(rcll_draw::C_WHITE);
    blbl_time_value.setBorderColor(rcll_draw::C_WHITE);
    blbl_score_value.setBorderColor(rcll_draw::C_WHITE);

    blbl_state_header.setFontSize(0.9);
    blbl_phase_header.setFontSize(0.9);
    blbl_time_header.setFontSize(0.9);
    blbl_score_header.setFontSize(0.9);
    blbl_state_value.setFontSize(0.9);
    blbl_phase_value.setFontSize(0.9);
    blbl_time_value.setFontSize(0.9);
    blbl_score_value.setFontSize(1.0);

    blbl_state_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_phase_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_time_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_score_header.setFrontColor(rcll_draw::C_WHITE);
    blbl_state_value.setFrontColor(rcll_draw::C_BLACK);
    blbl_phase_value.setFrontColor(rcll_draw::C_BLACK);
    blbl_time_value.setFrontColor(rcll_draw::C_BLACK);
    if (team == rcll_draw::CYAN){
        blbl_score_value.setFrontColor(rcll_draw::C_CYAN_DARK);
    } else if (team == rcll_draw::MAGENTA){
        blbl_score_value.setFrontColor(rcll_draw::C_MAGENTA_DARK);
    } else {
        blbl_score_value.setFrontColor(rcll_draw::C_BLACK);
    }
}

rcll_draw::HStatusPanel::~HStatusPanel(){

}

void rcll_draw::HStatusPanel::setGeometry(int x, int y, int w, int h){
    blbl_state_header.setPos(x, y);
    blbl_phase_header.setPos(x + w / 4, y);
    blbl_time_header.setPos(x + 2 * w / 4, y);
    blbl_score_header.setPos(x + 3 * w / 4, y);
    blbl_state_value.setPos(x, y + h / 2);
    blbl_phase_value.setPos(x + w / 4, y + h / 2);
    blbl_time_value.setPos(x + 2 * w / 4, y + h / 2);
    blbl_score_value.setPos(x + 3 * w / 4, y + h / 2);

    blbl_state_header.setSize(w / 4, h / 2);
    blbl_phase_header.setSize(w / 4, h / 2);
    blbl_time_header.setSize(w / 4, h / 2);
    blbl_score_header.setSize(w / 4, h / 2);
    blbl_state_value.setSize(w / 4, h / 2);
    blbl_phase_value.setSize(w / 4, h / 2);
    blbl_time_value.setSize(w / 4, h / 2);
    blbl_score_value.setSize(w / 4, h / 2);
}

void rcll_draw::HStatusPanel::setContent(std::string gamestate, std::string gamephase, int time, int score){
    int min = time / 60;
    int sec = time % 60;
    std::string time_str = std::to_string(min) + "min " + std::to_string(sec) + "sec";
    blbl_state_value.setContent(gamestate);
    blbl_phase_value.setContent(gamephase);
    blbl_time_value.setContent(time_str);
    blbl_score_value.setContent(std::to_string(score));
}

void rcll_draw::HStatusPanel::draw(cv::Mat &mat){
    blbl_state_header.draw(mat);
    blbl_phase_header.draw(mat);
    blbl_time_header.draw(mat);
    blbl_score_header.draw(mat);
    blbl_state_value.draw(mat);
    blbl_phase_value.draw(mat);
    blbl_time_value.draw(mat);
    blbl_score_value.draw(mat);
}

// ProductLabel ####################################################################

rcll_draw::ProductLabel::ProductLabel(){
    blbl_name.setAlignment(rcll_draw::CenterLeft);
    blbl_progress.setAlignment(rcll_draw::CenterLeft);
    blbl_deadline.setAlignment(rcll_draw::CenterLeft);
    blbl_points.setAlignment(rcll_draw::CenterLeft);

    blbl_name.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_progress.setBackgroundColor(rcll_draw::C_GREY_DARK);
    blbl_deadline.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_points.setBackgroundColor(rcll_draw::C_GREY_DARK);

    blbl_name.setBorderColor(rcll_draw::C_WHITE);
    blbl_progress.setBorderColor(rcll_draw::C_WHITE);
    blbl_deadline.setBorderColor(rcll_draw::C_WHITE);
    blbl_points.setBorderColor(rcll_draw::C_WHITE);

    blbl_name.setFontSize(0.9);
    blbl_progress.setFontSize(0.7);
    blbl_deadline.setFontSize(0.7);
    blbl_points.setFontSize(0.7);

    blbl_name.setFrontColor(rcll_draw::C_WHITE);
    blbl_progress.setFrontColor(rcll_draw::C_BLACK);
    blbl_deadline.setFrontColor(rcll_draw::C_BLACK);
    blbl_points.setFrontColor(rcll_draw::C_BLACK);

    img_step_progress.resize(5);
}

rcll_draw::ProductLabel::~ProductLabel(){

}

void rcll_draw::ProductLabel::setGeometry(int x, int y, int w, int h){
    blbl_name.setPos(x, y + h * 0.6);
    blbl_progress.setPos(x, y + h * 0.7);
    blbl_deadline.setPos(x, y + h * 0.8);
    blbl_points.setPos(x, y + h * 0.9);
    product.setPos(x, y + h * 0.07);

    blbl_name.setSize(w, h * 0.1);
    blbl_progress.setSize(w, h * 0.1);
    blbl_deadline.setSize(w, h * 0.1);
    blbl_points.setSize(w, h * 0.1);
    product.setScale(0.45);
    img_product_progress.setScale(0.2);

    for (size_t i = 0; i < img_step_progress.size(); i++){
        img_step_progress[i].setScale(0.175);
        img_step_progress[i].setPos(x + w * 0.6, y + h * 0.37 - i * h * 0.08);
    }

    img_product_progress.setPos(x + w * 0.2, y);
}

void rcll_draw::ProductLabel::setProduct(int id, Product plan, double progress, int deadline, int points, int points_max){
    int min = deadline / 60;
    int sec = deadline % 60;
    std::string time_str = std::to_string(min) + "min " + std::to_string(sec) + "sec";

    blbl_name.setContent(" Product P" + std::to_string(id));
    if (progress >= 1.0){
        blbl_progress.setContent(" Progress: finished");
    } else {
        blbl_progress.setContent(" Progress: " + std::to_string((int)(progress * 100)) + "%");
    }
    blbl_deadline.setContent(" Deadline at " + time_str);
    blbl_points.setContent(" Points: " + std::to_string(points) + " / " + std::to_string(points_max));
    product.setImage(createProductImage(plan));

    if (plan.complexity == 0){
        img_step_progress[0].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_base, 4)));
        img_step_progress[1].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_cap, 4)));
        img_step_progress[2].setImage(rcll_draw::readImage(""));
        img_step_progress[3].setImage(rcll_draw::readImage(""));
        img_step_progress[4].setImage(rcll_draw::readImage(""));
    } else if (plan.complexity == 1){
        img_step_progress[0].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_base, 4)));
        img_step_progress[1].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_ring1, 4)));
        img_step_progress[2].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_cap, 4)));
        img_step_progress[3].setImage(rcll_draw::readImage(""));
        img_step_progress[4].setImage(rcll_draw::readImage(""));
    } else if (plan.complexity == 2){
        img_step_progress[0].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_base, 4)));
        img_step_progress[1].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_ring1, 4)));
        img_step_progress[2].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_ring2, 4)));
        img_step_progress[3].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_cap, 4)));
        img_step_progress[4].setImage(rcll_draw::readImage(""));
    } else if (plan.complexity == 3){
        img_step_progress[0].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_base, 4)));
        img_step_progress[1].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_ring1, 4)));
        img_step_progress[2].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_ring2, 4)));
        img_step_progress[3].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_ring3, 4)));
        img_step_progress[4].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_base, 4)));
    }
    img_product_progress.setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_product, 4)));
}

cv::Mat rcll_draw::ProductLabel::createProductImage(rcll_draw::Product plan){
    cv::Mat img_bcg = rcll_draw::readImage("base_background.ppm");

    cv::Mat result(img_bcg.rows * 1.75, img_bcg.cols, CV_8UC4);
    cv::rectangle(result, cv::Point(0,0), cv::Point(result.cols, result.rows), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED, 8, 0);

    if (plan.complexity == 0){
        cv::Mat img_base = rcll_draw::readImage(rcll_draw::getFile(plan.base, 1));
        cv::Mat img_pos1 = rcll_draw::readImage(rcll_draw::getFile(plan.cap, 3));

        rcll_draw::mergeImages(result, img_bcg, rcll_draw::getColor(rcll_draw::C_WHITE), 0, result.rows - img_bcg.rows);
        rcll_draw::mergeImages(result, img_base, rcll_draw::getColor(rcll_draw::C_WHITE), (result.cols - img_base.cols) / 2, result.rows - img_base.rows - img_bcg.rows * 0.17);
        rcll_draw::mergeImages(result, img_pos1, rcll_draw::getColor(rcll_draw::C_WHITE), (result.cols - img_pos1.cols) / 2, result.rows - img_pos1.rows - img_bcg.rows * 0.63);
    } else if (plan.complexity == 1){
        cv::Mat img_base = rcll_draw::readImage(rcll_draw::getFile(plan.base, 1));
        cv::Mat img_pos1 = rcll_draw::readImage(rcll_draw::getFile(plan.ring1, 2));
        cv::Mat img_pos2 = rcll_draw::readImage(rcll_draw::getFile(plan.cap, 3));

        rcll_draw::mergeImages(result, img_bcg, rcll_draw::getColor(rcll_draw::C_WHITE), 0, result.rows - img_bcg.rows);
        rcll_draw::mergeImages(result, img_base, rcll_draw::getColor(rcll_draw::C_WHITE), (result.cols - img_base.cols) / 2, result.rows - img_base.rows - img_bcg.rows * 0.17);
        rcll_draw::mergeImages(result, img_pos1, rcll_draw::getColor(rcll_draw::C_WHITE), (result.cols - img_pos1.cols) / 2, result.rows - img_pos1.rows - img_bcg.rows * 0.63);
        rcll_draw::mergeImages(result, img_pos2, rcll_draw::getColor(rcll_draw::C_WHITE), (result.cols - img_pos2.cols) / 2, result.rows - img_pos2.rows - img_bcg.rows * 0.88);
    } else if (plan.complexity == 2){
        cv::Mat img_base = rcll_draw::readImage(rcll_draw::getFile(plan.base, 1));
        cv::Mat img_pos1 = rcll_draw::readImage(rcll_draw::getFile(plan.ring1, 2));
        cv::Mat img_pos2 = rcll_draw::readImage(rcll_draw::getFile(plan.ring2, 2));
        cv::Mat img_pos3 = rcll_draw::readImage(rcll_draw::getFile(plan.cap, 3));

        rcll_draw::mergeImages(result, img_bcg, rcll_draw::getColor(rcll_draw::C_WHITE), 0, result.rows - img_bcg.rows);
        rcll_draw::mergeImages(result, img_base, rcll_draw::getColor(rcll_draw::C_WHITE), (result.cols - img_base.cols) / 2, result.rows - img_base.rows - img_bcg.rows * 0.17);
        rcll_draw::mergeImages(result, img_pos1, rcll_draw::getColor(rcll_draw::C_WHITE), (result.cols - img_pos1.cols) / 2, result.rows - img_pos1.rows - img_bcg.rows * 0.63);
        rcll_draw::mergeImages(result, img_pos2, rcll_draw::getColor(rcll_draw::C_WHITE), (result.cols - img_pos2.cols) / 2, result.rows - img_pos2.rows - img_bcg.rows * 0.88);
        rcll_draw::mergeImages(result, img_pos3, rcll_draw::getColor(rcll_draw::C_WHITE), (result.cols - img_pos3.cols) / 2, result.rows - img_pos3.rows- img_bcg.rows * 1.09);
    } else if (plan.complexity == 3){
        cv::Mat img_base = rcll_draw::readImage(rcll_draw::getFile(plan.base, 1));
        cv::Mat img_pos1 = rcll_draw::readImage(rcll_draw::getFile(plan.ring1, 2));
        cv::Mat img_pos2 = rcll_draw::readImage(rcll_draw::getFile(plan.ring2, 2));
        cv::Mat img_pos3 = rcll_draw::readImage(rcll_draw::getFile(plan.ring3, 2));
        cv::Mat img_pos4 = rcll_draw::readImage(rcll_draw::getFile(plan.cap, 3));

        rcll_draw::mergeImages(result, img_bcg, rcll_draw::getColor(rcll_draw::C_WHITE), 0, result.rows - img_bcg.rows);
        rcll_draw::mergeImages(result, img_base, rcll_draw::getColor(rcll_draw::C_WHITE), (result.cols - img_base.cols) / 2, result.rows - img_base.rows - img_bcg.rows * 0.17);
        rcll_draw::mergeImages(result, img_pos1, rcll_draw::getColor(rcll_draw::C_WHITE), (result.cols - img_pos1.cols) / 2, result.rows - img_pos1.rows - img_bcg.rows * 0.63);
        rcll_draw::mergeImages(result, img_pos2, rcll_draw::getColor(rcll_draw::C_WHITE), (result.cols - img_pos2.cols) / 2, result.rows - img_pos2.rows - img_bcg.rows * 0.88);
        rcll_draw::mergeImages(result, img_pos3, rcll_draw::getColor(rcll_draw::C_WHITE), (result.cols - img_pos3.cols) / 2, result.rows - img_pos3.rows- img_bcg.rows * 1.09);
        rcll_draw::mergeImages(result, img_pos4, rcll_draw::getColor(rcll_draw::C_WHITE), (result.cols - img_pos4.cols) / 2, result.rows - img_pos4.rows - img_bcg.rows * 1.33);
    }
    return result;
}

void rcll_draw::ProductLabel::draw(cv::Mat &mat){
    blbl_name.draw(mat);
    blbl_progress.draw(mat);
    blbl_deadline.draw(mat);
    blbl_points.draw(mat);
    product.draw(mat);
    for (size_t i = 0; i < img_step_progress.size(); i++){
        img_step_progress[i].draw(mat);
    }
    img_product_progress.draw(mat);
}

// ProductInfo ####################################################################

rcll_draw::ProductInfo::ProductInfo(){
    products.resize(4);
}

rcll_draw::ProductInfo::~ProductInfo(){

}

void rcll_draw::ProductInfo::setGeometry(int x, int y, int w, int h, int gapsize){
    int w1 = (w - 3 * gapsize) / 4;
    products[0].setGeometry(x, y, w1, h);
    products[1].setGeometry(x + gapsize + w1, y, w1, h);
    products[2].setGeometry(x + 2 * (gapsize + w1), y, w1, h);
    products[3].setGeometry(x + 3 * (gapsize + w1), y, w1, h);
}

void rcll_draw::ProductInfo::setProduct(int id, Product plan, double progress, int deadline, int points, int points_max, int index){
    if (index >= 0 && index < 4){
        products[index].setProduct(id, plan, progress, deadline, points, points_max);
    }
}

void rcll_draw::ProductInfo::draw(cv::Mat &mat){
    products[0].draw(mat);
    products[1].draw(mat);
    products[2].draw(mat);
    products[3].draw(mat);
}


// MachineLabelExploration ####################################################################

rcll_draw::MachineLabelExploration::MachineLabelExploration(){
    blbl_machinename.setAlignment(rcll_draw::CenterLeft);
    blbl_status1.setAlignment(rcll_draw::CenterCenter);
    blbl_status2.setAlignment(rcll_draw::CenterCenter);

    blbl_machinename.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_status1.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_status2.setBackgroundColor(rcll_draw::C_GREY_LIGHT);

    blbl_machinename.setBorderColor(rcll_draw::C_WHITE);
    blbl_status1.setBorderColor(rcll_draw::C_WHITE);
    blbl_status2.setBorderColor(rcll_draw::C_WHITE);

    blbl_machinename.setFontSize(0.9);
    blbl_status1.setFontSize(0.7);
    blbl_status2.setFontSize(0.7);

    blbl_machinename.setFrontColor(rcll_draw::C_WHITE);
    blbl_status1.setFrontColor(rcll_draw::C_BLACK);
    blbl_status2.setFrontColor(rcll_draw::C_BLACK);

    img_status1.setImage(rcll_draw::readImage(rcll_draw::getFile(4,4)));
    img_status2.setImage(rcll_draw::readImage(rcll_draw::getFile(4,4)));
}

rcll_draw::MachineLabelExploration::~MachineLabelExploration(){

}

void rcll_draw::MachineLabelExploration::setGeometry(int x, int y, int w, int h){
    blbl_machinename.setSize(w * 0.4, h);
    blbl_status1.setSize(w * 0.3, h);
    blbl_status2.setSize(w * 0.3, h);
    img_status1.setScale(0.25);
    img_status2.setScale(0.25);

    blbl_machinename.setPos(x, y);
    blbl_status1.setPos(x + w * 0.4, y);
    blbl_status2.setPos(x + w * 0.7, y);
    img_status1.setPos(x + w * 0.625, y + (h - img_status1.getH())/2);
    img_status2.setPos(x + w * 0.925, y + (h - img_status2.getH())/2);
}

void rcll_draw::MachineLabelExploration::setMachineName(std::string name, int index){
    blbl_machinename.setContent(name);
    if (index % 2 == 0){
        blbl_status1.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
        blbl_status2.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    } else {
        blbl_status1.setBackgroundColor(rcll_draw::C_GREY_DARK);
        blbl_status2.setBackgroundColor(rcll_draw::C_GREY_DARK);
    }
}

void rcll_draw::MachineLabelExploration::setMachineStatus(int status1, int status2){
    if (status1 == 0){
        blbl_status1.setContent("unreported");
        img_status1.setImage(rcll_draw::readImage(rcll_draw::getFile(4,4)));
    } else if (status1 == 1){
        blbl_status1.setContent("correct");
        img_status1.setImage(rcll_draw::readImage(rcll_draw::getFile(5,4)));
    } else if (status1 == 2){
        blbl_status1.setContent("wrong");
        img_status1.setImage(rcll_draw::readImage(rcll_draw::getFile(6,4)));
    } else {
        blbl_status1.setContent("unknown");
        img_status1.setImage(rcll_draw::readImage(""));
    }

    if (status2 == 0){
        blbl_status2.setContent("unreported");
        img_status2.setImage(rcll_draw::readImage(rcll_draw::getFile(4,4)));
    } else if (status2 == 1){
        blbl_status2.setContent("correct");
        img_status2.setImage(rcll_draw::readImage(rcll_draw::getFile(5,4)));
    } else if (status2 == 2){
        blbl_status2.setContent("wrong");
        img_status2.setImage(rcll_draw::readImage(rcll_draw::getFile(6,4)));
    } else {
        blbl_status2.setContent("unknown");
        img_status2.setImage(rcll_draw::readImage(""));
    }
}

void rcll_draw::MachineLabelExploration::setHeader(std::string status1, std::string status2){
    blbl_machinename.setContent("");
    blbl_status1.setContent(status1);
    blbl_status2.setContent(status2);

    blbl_machinename.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_status1.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_status2.setBackgroundColor(rcll_draw::C_BLACK);

    blbl_machinename.setFrontColor(rcll_draw::C_WHITE);
    blbl_status1.setFrontColor(rcll_draw::C_WHITE);
    blbl_status2.setFrontColor(rcll_draw::C_WHITE);

    blbl_machinename.setFontSize(1.0);
    blbl_status1.setFontSize(1.0);
    blbl_status2.setFontSize(1.0);

    img_status1.setImage(rcll_draw::readImage(""));
    img_status2.setImage(rcll_draw::readImage(""));
}

void rcll_draw::MachineLabelExploration::draw(cv::Mat &mat){
    blbl_machinename.draw(mat);
    blbl_status1.draw(mat);
    blbl_status2.draw(mat);
    img_status1.draw(mat, rcll_draw::getColor(rcll_draw::C_WHITE));
    img_status2.draw(mat, rcll_draw::getColor(rcll_draw::C_WHITE));
}


// MachineLabelProduction ####################################################################

rcll_draw::MachineLabelProduction::MachineLabelProduction(){
    blbl_machinename.setAlignment(rcll_draw::CenterLeft);
    blbl_state.setAlignment(rcll_draw::CenterCenter);
    blbl_lamp1.setAlignment(rcll_draw::CenterCenter);
    blbl_lamp2.setAlignment(rcll_draw::CenterCenter);

    blbl_machinename.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_state.setBackgroundColor(rcll_draw::C_TRANSPARENT);
    blbl_lamp1.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_lamp2.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_border.setBackgroundColor(rcll_draw::C_TRANSPARENT);

    blbl_machinename.setBorderColor(rcll_draw::C_TRANSPARENT);
    blbl_state.setBorderColor(rcll_draw::C_TRANSPARENT);
    blbl_lamp1.setBorderColor(rcll_draw::C_WHITE);
    blbl_lamp2.setBorderColor(rcll_draw::C_WHITE);
    blbl_border.setBorderColor(rcll_draw::C_WHITE);

    blbl_machinename.setFontSize(0.9);
    blbl_state.setFontSize(0.7);
    blbl_lamp1.setFontSize(0.7);
    blbl_lamp2.setFontSize(0.7);

    blbl_machinename.setFrontColor(rcll_draw::C_WHITE);
    blbl_state.setFrontColor(rcll_draw::C_BLACK);
    blbl_lamp1.setFrontColor(rcll_draw::C_BLACK);
    blbl_lamp2.setFrontColor(rcll_draw::C_BLACK);
}

rcll_draw::MachineLabelProduction::~MachineLabelProduction(){

}

void rcll_draw::MachineLabelProduction::setGeometry(int x, int y, int w, int h){
    blbl_machinename.setPos(x, y);
    blbl_state.setPos(x + w * 0.7, y);
    blbl_lamp1.setPos(x + w * 0.7, y);
    blbl_lamp2.setPos(x + w * 0.7, y + h/2);
    blbl_border.setPos(x, y);

    blbl_machinename.setSize(w * 0.7, h);
    blbl_state.setSize(w * 0.3, h);
    blbl_lamp1.setSize(w * 0.3, h/2);
    blbl_lamp2.setSize(w * 0.3, h/2);
    blbl_border.setSize(w, h);
}

void rcll_draw::MachineLabelProduction::setMachineName(std::string name){
    blbl_machinename.setContent(name);
}

void rcll_draw::MachineLabelProduction::setMachineStatus(std::string status, Color lamp_top, Color lamp_bottom, std::string lamp_top_str, std::string lamp_bottom_str){
    blbl_state.setContent(status);
    blbl_lamp1.setContent(lamp_top_str);
    blbl_lamp2.setContent(lamp_bottom_str);
    lamp1_color = lamp_top;
    lamp2_color = lamp_bottom;
    blbl_lamp1.setBackgroundColor(lamp1_color);
    blbl_lamp2.setBackgroundColor(lamp2_color);
}

void rcll_draw::MachineLabelProduction::setFlashing(bool flashing){
    this->flashing = flashing;
}

void rcll_draw::MachineLabelProduction::draw(cv::Mat &mat){
    if (flashing){
        if (getBoolSignal(ros::Time::now(), ros::Rate(1.33))){
            blbl_lamp1.setBackgroundColor(lamp1_color);
            blbl_lamp2.setBackgroundColor(lamp2_color);
            blbl_lamp1.setBorderColor(lamp1_color);
            blbl_lamp2.setBorderColor(lamp2_color);
        } else {
            blbl_lamp1.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
            blbl_lamp2.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
            blbl_lamp1.setBorderColor(rcll_draw::C_GREY_LIGHT);
            blbl_lamp2.setBorderColor(rcll_draw::C_GREY_LIGHT);
        }
    } else {
        blbl_lamp1.setBackgroundColor(lamp1_color);
        blbl_lamp2.setBackgroundColor(lamp2_color);
        blbl_lamp1.setBorderColor(lamp1_color);
        blbl_lamp2.setBorderColor(lamp2_color);
    }

    blbl_machinename.draw(mat);
    blbl_lamp1.draw(mat);
    blbl_lamp2.draw(mat);
    blbl_state.draw(mat);
    blbl_border.draw(mat);
}

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

void rcll_draw::MachineInfoExploration::setMachineName(std::string name, int index){
    if (index >= 0 && index < 7){
        machines[index + 1].setMachineName(" " + name, index + 1);
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
            lamp1 = rcll_draw::C_GREEN;
            lamp2 = rcll_draw::C_GREEN;
            lamp1_str = "Free For";
            lamp2_str = "Production";
        } else if (status == "Broken"){
            lamp1 = rcll_draw::C_RED;
            lamp2 = rcll_draw::C_YELLOW;
            flashing = true;
            lamp1_str = "Incorrect";
            lamp2_str = "Instruction";
        } else if (status == "Processing"){
            lamp1 = rcll_draw::C_GREEN;
            lamp2 = rcll_draw::C_YELLOW;
            lamp1_str = "Processing";
            lamp2_str = "Product";
        } else if (status == "Prepared"){
            lamp1 = rcll_draw::C_GREEN;
            lamp2 = rcll_draw::C_GREEN;
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
        blbl_name.setFrontColor(rcll_draw::C_GREEN);
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

void rcll_draw::RobotInfo::setRobotName(std::string name_str, bool active, int index){
    if (index >= 0 && index < 3){
        robots[index].setRobotName(" " + name_str, active);
    }
}

void rcll_draw::RobotInfo::setRobotStatus(std::string activity, double active_time, int maintenance, int maintenance_max, int index){
    if (index >= 0 && index < 3){
        robots[index].setRobotStatus(activity, active_time, maintenance, maintenance_max);
    }
}

void rcll_draw::RobotInfo::draw(cv::Mat &mat){
    blbl_header.draw(mat);
    robots[0].draw(mat);
    robots[1].draw(mat);
    robots[2].draw(mat);
}

// TeamAreaProduction ####################################################################

rcll_draw::TeamAreaProduction::TeamAreaProduction(rcll_draw::Team team){
    game_info = VStatusPanel(team);
    product_info = ProductInfo();
    machine_info = MachineInfoProduction(team);
    robot_info = RobotInfo(team);
}

rcll_draw::TeamAreaProduction::~TeamAreaProduction(){

}

void rcll_draw::TeamAreaProduction::setGeometry(int x, int y, int w, int h, int gapsize){
    int w1 = (w - gapsize);
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
    game_info.setGeometry(x, y, w1 * 0.12, h * 0.6);
    product_info.setGeometry(x + gapsize + w1 * 0.12, y, w1 * 0.88, h * 0.6, gapsize);
    machine_info.setGeometry(x, y + h * 0.6, w1 * 0.55, h * 0.4, gapsize);
    robot_info.setGeometry(x + w1 * 0.55 + gapsize, y + h * 0.6, w1 * 0.45, h * 0.4, gapsize);
}

void rcll_draw::TeamAreaProduction::setGameInfo(std::string gamestate, std::string gamephase, int time, int score){
    game_info.setContent(gamestate, gamephase, time, score);
}

void rcll_draw::TeamAreaProduction::setMachineName(std::string name, int index){
    machine_info.setMachineName(name, index);
}

void rcll_draw::TeamAreaProduction::setMachineStatus(std::string status, int index){
    machine_info.setMachineStatus(status, index);
}

void rcll_draw::TeamAreaProduction::setRobotName(std::string name, bool active, int index){
    robot_info.setRobotName(name, active, index);
}

void rcll_draw::TeamAreaProduction::setRobotStatus(std::string activity, double active_time, int maintenance_count, int maintenance_max, int index){
    robot_info.setRobotStatus(activity, active_time, maintenance_count, maintenance_max, index);
}

void rcll_draw::TeamAreaProduction::setProduct(int id, Product plan, double progress, int deadline, int points, int points_max, int index){
    product_info.setProduct(id, plan, progress, deadline, points, points_max, index);
}

void rcll_draw::TeamAreaProduction::draw(cv::Mat &mat){
    game_info.draw(mat);
    product_info.draw(mat);
    machine_info.draw(mat);
    robot_info.draw(mat);
}

// TeamAreaExploration ####################################################################

rcll_draw::TeamAreaExploration::TeamAreaExploration(rcll_draw::Team team){
    game_info = HStatusPanel(team);
    machine_info = MachineInfoExploration(team);
}

rcll_draw::TeamAreaExploration::~TeamAreaExploration(){

}

void rcll_draw::TeamAreaExploration::setGeometry(int x, int y, int w, int h, int gapsize){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
    game_info.setGeometry(x + w * 0.2, y, w * 0.6, h * 0.1);
    machine_info.setGeometry(x + w * 0.2, y + h * 0.2, w * 0.6, h * 0.8, gapsize);
}

void rcll_draw::TeamAreaExploration::setGameInfo(std::string gamestate, std::string gamephase, int time, int score){
    game_info.setContent(gamestate, gamephase, time, score);
}

void rcll_draw::TeamAreaExploration::setMachineName(std::string name, int index){
    machine_info.setMachineName(name, index);
}

void rcll_draw::TeamAreaExploration::setMachineStatus(int status1, int status2, int index){
    machine_info.setMachineStatus(status1, status2, index);
}

void rcll_draw::TeamAreaExploration::draw(cv::Mat &mat){
    game_info.draw(mat);
    machine_info.draw(mat);
}

// ??? ####################################################################

#endif
