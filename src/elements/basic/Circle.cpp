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
        cv::circle(mat, cv::Point(x, y), r, getColor(backgroundcolor), cv::FILLED, 8, 0);
    }
    if (bordercolor != rcll_draw::C_TRANSPARENT){
        // draw border circle
        cv::circle(mat, cv::Point(x, y), r, getColor(bordercolor), bordersize, 8, 0);
    }
}
