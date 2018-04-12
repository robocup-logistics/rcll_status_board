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

#ifndef RCLL_LINE_H
#define RCLL_LINE_H

#include <util.h>

namespace rcll_draw {
    class Line {
    public:
        Line();
        ~Line();
        void setLineByPoints(int x1, int y1, int x2, int y2);
        void setLineByLength(int x1, int y1, double angle, double length);
        void setBorderColor(Color c);
        void setBorderSize(int bordersize);
        void setLineType(LineType linetype);
        void draw(cv::Mat &mat);

    protected:
        int x1, y1, x2, y2 = 0;
        Color bordercolor = C_BLACK;
        int bordersize = 1;
        LineType linetype = Continuous;
    };

    class HLine : public Line {
    public:
        HLine();
        ~HLine();

        void setLine(int x1, int y, int x2);
    };

    class VLine : public Line {
    public:
        VLine();
        ~VLine();

        void setLine(int x, int y1, int y2);
    };
}

#endif
