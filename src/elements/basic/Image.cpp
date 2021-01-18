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

#include <Image.h>

// Image functions #################################################
rcll_draw::Image::Image(){
    this->image = cv::Mat(0, 0, CV_8UC4);
}

rcll_draw::Image::~Image(){

}

void rcll_draw::Image::setPos(int x, int y){
    this->x = x;
    this->y = y;
}

void rcll_draw::Image::setScale(double s){
    this->s = s;
}

int rcll_draw::Image::getW(){
    return (double)this->image.cols * s;
}

int rcll_draw::Image::getH(){
    return (double)this->image.rows * s;
}

void rcll_draw::Image::setImage(cv::Mat image){
    this->path = "";
    this->image = image;
}

void rcll_draw::Image::loadImage(std::string file){
    this->path = rcll_draw::getImagePath() + file;

    this->image = cv::imread(path, cv::IMREAD_UNCHANGED);
    if(!this->image.data){
        ROS_WARN("Could not open or find the picture at '%s'", path.c_str());
        return;
    }
}

void rcll_draw::Image::setAngle(double angle){
    this->angle = angle;
}

void rcll_draw::Image::draw(cv::Mat &mat){
    rcll_draw::mergeImages(mat, image, x, y, s);
}

void rcll_draw::Image::draw(cv::Mat &mat, cv::Scalar alpha_color){
    rcll_draw::mergeImages(mat, image, alpha_color, x, y, s);
}
