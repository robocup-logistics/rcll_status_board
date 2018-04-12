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

#include <Label.h>

// Label functions #################################################
rcll_draw::Label::Label(){

}

rcll_draw::Label::~Label(){

}

void rcll_draw::Label::setContent(std::string content){
    this->content = content;
}

void rcll_draw::Label::setPos(int x, int y){
    this->x = x;
    this->y = y;
}

void rcll_draw::Label::setFrontColor(Color c){
    this->frontcolor = c;
}

void rcll_draw::Label::setFont(std::string font, double fontsize){
    this->font = font;
    this->fontsize = fontsize;
}

void rcll_draw::Label::setFontSize(double fontsize){
    this->fontsize = fontsize;
}

void rcll_draw::Label::setAlignment(Alignment alignment){
    this->alignment = alignment;
}

void rcll_draw::Label::draw(cv::Mat &mat){
    int baseline = 0;
    cv::Size textsize = cv::getTextSize(content, BASIC_FONT, fontsize, 2 * fontsize, &baseline);
    cv::Point org;
    if (alignment == Alignment::TopLeft){
        org = cv::Point(x, y + textsize.height);
    } else if (alignment == Alignment::TopCenter){
        org = cv::Point(x - textsize.width / 2, y + textsize.height);
    } else if (alignment == Alignment::TopRight){
        org = cv::Point(x - textsize.width, y + textsize.height);
    } else if (alignment == Alignment::CenterLeft){
        org = cv::Point(x, y + textsize.height / 2);
    } else if (alignment == Alignment::CenterCenter){
        org = cv::Point(x - textsize.width / 2, y + textsize.height / 2);
    } else if (alignment == Alignment::CenterRight){
        org = cv::Point(x - textsize.width, y + textsize.height / 2);
    } else if (alignment == Alignment::BottomLeft){
        org = cv::Point(x, y);
    } else if (alignment == Alignment::BottomCenter){
        org = cv::Point(x - textsize.width / 2, y);
    } else if (alignment == Alignment::BottomRight){
        org = cv::Point(x - textsize.width, y);
    } else {
        org = cv::Point(0, 0);
    }
    cv::putText(mat, content, org, BASIC_FONT, fontsize, getColor(frontcolor), 2 * fontsize, 8, false);
}
