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

#include <Line.h>

// Line functions #################################################
rcll_draw::Line::Line(){

}

rcll_draw::Line::~Line(){

}

void rcll_draw::Line::setLineByPoints(int x1, int y1, int x2, int y2){
    this->x1 = x1;
    this->y1 = y1;
    this->x2 = x2;
    this->y2 = y2;
}

void rcll_draw::Line::setLineByLength(int x1, int y1, double angle, double length){
    this->x1 = x1;
    this->y1 = y1;
    this->x2 = x1 + length * cos(angle);
    this->y2 = y1 + length * sin(angle);
}

void rcll_draw::Line::setBorderColor(Color c){
    this->bordercolor = c;
}

void rcll_draw::Line::setBorderSize(int bordersize){
    this->bordersize = bordersize;
}

void rcll_draw::Line::setLineType(LineType linetype){
    this->linetype = linetype;
}

void rcll_draw::Line::draw(cv::Mat &mat){
    if (linetype == LineType::Continuous){
        cv::line(mat, cv::Point(x1, y1), cv::Point(x2, y2), getColor(bordercolor), bordersize, 8, 0);
    } else if (linetype == LineType::Dotted){
        // TODO
    } else if (linetype == LineType::Dashed){
        // TODO
    } else if (linetype == LineType::Arrowed){
        // TODO
    }

}


// HLine functions #################################################
rcll_draw::HLine::HLine(){

}

rcll_draw::HLine::~HLine(){

}

void rcll_draw::HLine::setLine(int x1, int y, int x2){
    Line::setLineByPoints(x1, y, x2, y);
}


// VLine functions #################################################
rcll_draw::VLine::VLine(){

}

rcll_draw::VLine::~VLine(){

}

void rcll_draw::VLine::setLine(int x, int y1, int y2){
    Line::setLineByPoints(x, y1, x, y2);
}
