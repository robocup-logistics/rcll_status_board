#ifndef RCLL_ARROW_H
#define RCLL_ARROW_H

#include <util.h>

namespace rcll_draw {
    class Arrow {
    public:
        Arrow();
        ~Arrow();
        void setArrowByLength(int x1, int y1, double angle, double length);
        void setColor(Color c);
        void draw(cv::Mat &mat);

    protected:
        std::vector<cv::Point> polygon;
        Color color = C_BLACK;
    };
}

#endif
