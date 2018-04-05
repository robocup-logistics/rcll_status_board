#include <ProductLabel.h>

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

    blbl_name.setFontSize(1.0);
    blbl_progress.setFontSize(0.8);
    blbl_deadline.setFontSize(0.8);
    blbl_points.setFontSize(0.8);

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
    img_product_progress.setScale(0.25);

    for (size_t i = 0; i < img_step_progress.size(); i++){
        img_step_progress[i].setScale(0.175);
        img_step_progress[i].setPos(x + w * 0.6, y + h * 0.37 - i * h * 0.08);
    }

    img_product_progress.setPos(x + w * 0.2, y - h * 0.03);
}

void rcll_draw::ProductLabel::setProduct(int id, Product plan, double progress, int deadline, int points, int points_max){
    int min = deadline / 60;
    int sec = deadline % 60;
    std::string time_str = std::to_string(min) + "min " + std::to_string(sec) + "sec";


    this->id = id;
    blbl_name.setContent(" Product P" + std::to_string(id));
    if (progress >= 1.0){
        blbl_progress.setContent(" Progress: finished");
    } else {
        blbl_progress.setContent(" Progress: " + std::to_string((int)(round(progress * 100.0))) + "%");
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
        img_step_progress[4].setImage(rcll_draw::readImage(rcll_draw::getFile(plan.status_cap, 4)));
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
    if (id > 0){
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
}
