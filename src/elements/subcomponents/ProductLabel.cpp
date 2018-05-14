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

#include <ProductLabel.h>

// ProductLabel ####################################################################

rcll_draw::ProductLabel::ProductLabel(){
    origin = cv::Mat(h, w, CV_8UC4);

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

void rcll_draw::ProductLabel::setGeometry(int x, int y){
    this->x = x;
    this->y = y;

    blbl_name.setPos(0, h * 0.6);
    blbl_progress.setPos(0, h * 0.7);
    blbl_deadline.setPos(0, h * 0.8);
    blbl_points.setPos(0, h * 0.9);
    img_product.setPos(w * 0.2, h * 0.1);

    blbl_name.setSize(w, h * 0.1);
    blbl_progress.setSize(w, h * 0.1);
    blbl_deadline.setSize(w, h * 0.1);
    blbl_points.setSize(w, h * 0.1);
    img_product.setScale(0.45);
    img_product_progress.setScale(0.25);

    for (size_t i = 0; i < img_step_progress.size(); i++){
        img_step_progress[i].setScale(0.175);
        img_step_progress[i].setPos(w * 0.8, h * 0.4 - i * h * 0.08);
    }

    img_product_progress.setPos(w * 0.4, 0);
}

int rcll_draw::ProductLabel::getW(){
    return w;
}

int rcll_draw::ProductLabel::getH(){
    return h;
}

void rcll_draw::ProductLabel::setProduct(rcll_vis_msgs::Product &product, rcll_draw::Team team){
    if (product.product_id == 0){
        return;
    }

    int min = product.end_delivery_time / 60;
    int sec = product.end_delivery_time % 60;
    std::string time_str = std::to_string(min) + "min " + std::to_string(sec) + "sec";
    blbl_name.setContent(" Product P" + std::to_string(product.product_id) + "." + std::to_string(product.quantity_id));
    if (team == rcll_draw::CYAN){
        if (product.progress_cyan >= 1.0){
            blbl_progress.setContent(" Progress: finished");
        } else {
            blbl_progress.setContent(" Progress: " + std::to_string((int)(round(product.progress_cyan * 100.0))) + "%");
        }
        blbl_points.setContent(" Points: " + std::to_string(product.points_cyan) + " / " + std::to_string(product.points_max));
    } else if (team == rcll_draw::MAGENTA){
        if (product.progress_magenta >= 1.0){
            blbl_progress.setContent(" Progress: finished");
        } else {
            blbl_progress.setContent(" Progress: " + std::to_string((int)(round(product.progress_magenta * 100.0))) + "%");
        }
        blbl_points.setContent(" Points: " + std::to_string(product.points_magenta) + " / " + std::to_string(product.points_max));
    }

    blbl_deadline.setContent(" Deadline at " + time_str);

    ProductPlan plan;

    plan.complexity = product.complexity;
    plan.base = product.structure[0];
    plan.ring1 = product.structure[1];
    plan.ring2 = product.structure[2];
    plan.ring3 = product.structure[3];
    plan.cap = product.structure[4];

    if (team == rcll_draw::CYAN){
        plan.status_base = product.step_stati_cyan[0];
        plan.status_ring1 = product.step_stati_cyan[1];
        plan.status_ring2 = product.step_stati_cyan[2];
        plan.status_ring3 = product.step_stati_cyan[3];
        plan.status_cap = product.step_stati_cyan[4];
        plan.status_product = product.step_stati_cyan[5];
    } else if (team == rcll_draw::MAGENTA){
        plan.status_base = product.step_stati_magenta[0];
        plan.status_ring1 = product.step_stati_magenta[1];
        plan.status_ring2 = product.step_stati_magenta[2];
        plan.status_ring3 = product.step_stati_magenta[3];
        plan.status_cap = product.step_stati_magenta[4];
        plan.status_product = product.step_stati_magenta[5];
    }
    img_product.setImage(createProductImage(plan));

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

cv::Mat rcll_draw::ProductLabel::createProductImage(rcll_draw::ProductPlan plan){
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
    cv::rectangle(origin, cv::Point(0, 0), cv::Point (w-1, h-1), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED);

    for (size_t i = 0; i < img_step_progress.size(); i++){
        img_step_progress[i].draw(origin);
    }
    img_product_progress.draw(origin);

    img_product.draw(origin);

    blbl_name.draw(origin);
    blbl_progress.draw(origin);
    blbl_deadline.draw(origin);
    blbl_points.draw(origin);

    rcll_draw::mergeImages(mat, origin, x, y);
}
