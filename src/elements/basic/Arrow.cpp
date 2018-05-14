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

#include <Arrow.h>

// Arrow functions #################################################
rcll_draw::Arrow::Arrow(){

}

rcll_draw::Arrow::~Arrow(){

}

void rcll_draw::Arrow::setArrowByLength(int x1, int y1, double angle, double length){
    cv::Point p;
    polygon.clear();

    /*
              p3
              *
             / \
            /   \
           *--*--*
         p4  p1  p2
    */

    // Rotational CenterPoint p1
    p.x = x1;
    p.y = y1;
    polygon.push_back(p);

    // Bottom Right Point p2
    p.x = x1 + length / 5 * cos(angle);
    p.y = y1 + length / 5 * sin(angle);
    polygon.push_back(p);

    // Top Center Point p3
    p.x = x1 + length * cos(angle + M_PI / 2);
    p.y = y1 + length * sin(angle + M_PI / 2);
    polygon.push_back(p);

    // Bottom Left Point p4
    p.x = x1 - length / 5 * cos(angle);
    p.y = y1 - length / 5 * sin(angle);
    polygon.push_back(p);
}

void rcll_draw::Arrow::setColor(Color c){
    this->color = c;
}

void rcll_draw::Arrow::draw(cv::Mat &mat){
    cv::fillConvexPoly(mat, this->polygon, rcll_draw::getColor(this->color), 8, 0);
}
