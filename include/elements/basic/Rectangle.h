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

#ifndef RCLL_RECTANGLE_H
#define RCLL_RECTANGLE_H

#include <util.h>
#include <Label.h>

namespace rcll_draw {
    class Rectangle {
    public:
        Rectangle();
        ~Rectangle();

        void setPos(int x, int y); // origin is top left corner
        void setSize(int w, int h);
        void setBackgroundColor(Color c);
        void setBorderColor(Color c);
        void setBorderSize(int bordersize);
        void draw(cv::Mat &mat);

    protected:
        int x, y = 0;
        int w, h = 10;
        Color bordercolor = C_BLACK;
        Color backgroundcolor = C_WHITE;
        int bordersize = 1;

    };
}

#endif
