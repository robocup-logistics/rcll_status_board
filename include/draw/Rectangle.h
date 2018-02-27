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
