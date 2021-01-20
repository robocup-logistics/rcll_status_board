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

#include <BoxLabel.h>

// BoxLabel functions #################################################
rcll_draw::BoxLabel::BoxLabel(){

}

rcll_draw::BoxLabel::~BoxLabel(){

}

void rcll_draw::BoxLabel::setSize(int w, int h){
    this->w = w;
    this->h = h;
}

int rcll_draw::BoxLabel::getW(){
    return w;
}

int rcll_draw::BoxLabel::getH(){
    return h;
}

void rcll_draw::BoxLabel::setBackgroundColor(Color c){
    this->backgroundcolor = c;
}

void rcll_draw::BoxLabel::setBorderColor(Color c){
    this->bordercolor = c;
}

void rcll_draw::BoxLabel::setBorderSize(int bordersize){
    this->bordersize = bordersize;
}

void rcll_draw::BoxLabel::draw(cv::Mat &mat){
    if (backgroundcolor != rcll_draw::C_TRANSPARENT){
        // draw background rectangle
        cv::rectangle(mat, cv::Point(x, y), cv::Point(x + w, y + h), getColor(backgroundcolor), cv::FILLED, 8, 0);
    }

    if (bordercolor != rcll_draw::C_TRANSPARENT){
        // draw border rectangle
        cv::rectangle(mat, cv::Point(x, y), cv::Point(x + w, y + h), getColor(bordercolor), bordersize, 8, 0);
    }

    if (content != ""){
        int baseline = 0;
        cv::Size textsize = cv::getTextSize(content, BASIC_FONT, fontsize, 2 * fontsize, &baseline);
        cv::Point org;
        if (alignment == Alignment::TopLeft){
            org = cv::Point(x, y + textsize.height + h*0.15);
        } else if (alignment == Alignment::TopCenter){
            org = cv::Point(x + (w - textsize.width) / 2, y + textsize.height + h*0.15);
        } else if (alignment == Alignment::TopRight){
            org = cv::Point(x + w - textsize.width, y + textsize.height + h*0.15);
        } else if (alignment == Alignment::CenterLeft){
            org = cv::Point(x, y + (h + textsize.height) / 2);
        } else if (alignment == Alignment::CenterCenter){
            org = cv::Point(x + (w - textsize.width) / 2, y + (h + textsize.height) / 2);
        } else if (alignment == Alignment::CenterRight){
            org = cv::Point(x + w - textsize.width, y + (h + textsize.height) / 2);
        } else if (alignment == Alignment::BottomLeft){
            org = cv::Point(x, y + h - h*0.15);
        } else if (alignment == Alignment::BottomCenter){
            org = cv::Point(x + (w - textsize.width) / 2, y + h - h*0.15);
        } else if (alignment == Alignment::BottomRight){
            org = cv::Point(x + w - textsize.width, y + h - h*0.15);
        } else {
            org = cv::Point(0, 0);
        }
        cv::putText(mat, content, org, BASIC_FONT, fontsize, getColor(frontcolor), 2 * fontsize, 8, false);
    }
}
