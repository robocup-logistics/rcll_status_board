#include <Rectangle.h>

// Rectangle functions #################################################
rcll_draw::Rectangle::Rectangle(){

}

rcll_draw::Rectangle::~Rectangle(){

}

void rcll_draw::Rectangle::setPos(int x, int y){
    this->x = x;
    this->y = y;
}

void rcll_draw::Rectangle::setSize(int w, int h){
    this->w = w;
    this->h = h;
}

void rcll_draw::Rectangle::setBackgroundColor(Color c){
    this->backgroundcolor = c;
}

void rcll_draw::Rectangle::setBorderColor(Color c){
    this->bordercolor = c;
}

void rcll_draw::Rectangle::setBorderSize(int bordersize){
    this->bordersize = bordersize;
}

void rcll_draw::Rectangle::draw(cv::Mat &mat){
    if (backgroundcolor != rcll_draw::C_TRANSPARENT){
        // draw background rectangle
        cv::rectangle(mat, cv::Point(x, y), cv::Point(x + w, y + h), getColor(backgroundcolor), CV_FILLED, 8, 0);
    }

    if (bordercolor != rcll_draw::C_TRANSPARENT){
        // draw border rectangle
        cv::rectangle(mat, cv::Point(x, y), cv::Point(x + w, y + h), getColor(bordercolor), bordersize, 8, 0);
    }
}
