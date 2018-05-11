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

#include <ProductInfo.h>

// ProductInfo ####################################################################
rcll_draw::ProductInfo::ProductInfo() : rcll_draw::ProductInfo::ProductInfo(rcll_draw::NO_TEAM){

}

rcll_draw::ProductInfo::ProductInfo(rcll_draw::Team team, int displayed_products){
    this->displayed_products = displayed_products;
    this->team = team;
    w = (displayed_products * (w_product + gapsize)) - gapsize;
    origin = cv::Mat(h, w, CV_8UC4);
}

rcll_draw::ProductInfo::~ProductInfo(){

}

void rcll_draw::ProductInfo::setGeometry(int x, int y, double scale){
    this->x = x;
    this->y = y;
    this->scale = scale;
}

int rcll_draw::ProductInfo::getW(double scale){
    return (int)((double)w * scale);
}

int rcll_draw::ProductInfo::getH(double scale){
    return (int)((double)h * scale);
}

void rcll_draw::ProductInfo::setProducts(std::vector<rcll_vis_msgs::Product> &products){
    for (size_t i = 0; i < products.size(); i++){
        std::string key = std::to_string(products[i].product_id) + "-" + std::to_string(products[i].quantity_id);
        pls_products[key].setProduct(products[i], team);
    }
    getKeys(pls_products, keys);
    display = keys;

    prepare_draw();
}

void rcll_draw::ProductInfo::prepare_draw(){
    if (display.size() > 0){
        int canvas_w = (display.size() * (w_product + gapsize)) - gapsize;
        canvas = cv::Mat(h, canvas_w, CV_8UC4);
        cv::rectangle(canvas, cv::Point(0, 0), cv::Point (canvas_w-1, h-1), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED);

        int cur_x = 0;
        for (size_t i = 0; i < display.size(); i++){
            pls_products[display[i]].setGeometry(cur_x, 0);
            pls_products[display[i]].draw(canvas);
            cur_x += pls_products[display[i]].getW() + gapsize;
        }
    }
}

bool rcll_draw::ProductInfo::move(){
    if (display.size() > displayed_products){
        if (shift >= (int)((display.size() - displayed_products) * w_product)){
            if (wait < 10){
                wait++;
                return true;
            } else {
                wait=0;
                shift=0;
                return false;
            }
        } else if(shift == 0){
            if (wait < 10){
                wait++;
                return true;
            } else {
                wait=0;
                shift+=10;
                return true;
            }
        } else {
            shift+=10;
            return true;
        }
    } else {
        shift = 0;
        return false;
    }
}

void rcll_draw::ProductInfo::draw(cv::Mat &mat, bool show_element_border){
    if (display.size() > 0){
        cv::rectangle(origin, cv::Point(0, 0), cv::Point (w-1, h-1), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED);

        cv::Rect roi = cv::Rect(shift, 0, std::min(w, canvas.cols - shift), h);
        crop = canvas(roi);

        rcll_draw::mergeImages(origin, crop, 0, 0);
    }

    if (show_element_border){
        cv::rectangle(origin, cv::Point(0, 0), cv::Point (w-1, h-1), rcll_draw::getColor(rcll_draw::C_RED), 1);
    }

    rcll_draw::mergeImages(mat, origin, x, y, scale);
}

void rcll_draw::ProductInfo::getKeys(std::map<std::string, rcll_draw::ProductLabel> &mapping, std::vector<std::string> &keys){
    keys.clear();
    for (std::map<std::string, rcll_draw::ProductLabel>::iterator it = mapping.begin(); it != mapping.end(); ++it) {
        keys.push_back(it->first);
    }
}
