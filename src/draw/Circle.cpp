#include <Circle.h>

// Circle functions #################################################
rcll_draw::Circle::Circle(){

}

rcll_draw::Circle::~Circle(){

}

void rcll_draw::Circle::setPos(int x, int y){
    this->x = x;
    this->y = y;
}

void rcll_draw::Circle::setSize(double radius){
    this->r = radius;
}

void rcll_draw::Circle::setBackgroundColor(Color c){
    this->backgroundcolor = c;
}

void rcll_draw::Circle::setBorderColor(Color c){
    this->bordercolor = c;
}

void rcll_draw::Circle::setBorderSize(int bordersize){
    this->bordersize = bordersize;
}

void rcll_draw::Circle::draw(cv::Mat &mat){
    if (backgroundcolor != rcll_draw::C_TRANSPARENT){
        // draw background circle
        cv::circle(mat, cv::Point(x, y), r, getColor(backgroundcolor), CV_FILLED, 8, 0);
    }
    if (bordercolor != rcll_draw::C_TRANSPARENT){
        // draw border circle
        cv::circle(mat, cv::Point(x, y), r, getColor(bordercolor), bordersize, 8, 0);
    }
}
