#ifndef RCLL_CIRCLE_H
#define RCLL_CIRCLE_H

#include <util.h>

namespace rcll_draw {
    class Circle {
    public:
        Circle();
        ~Circle();

        void setPos(int x, int y); // origin is center or circle
        void setSize(double radius);
        void setBackgroundColor(Color c);
        void setBorderColor(Color c);
        void setBorderSize(int bordersize);
        void draw(cv::Mat &mat);

    protected:
        int x, y = 0;
        double r = 1;
        Color bordercolor = C_BLACK;
        Color backgroundcolor = C_WHITE;
        int bordersize = 1;
    };
}

#endif
