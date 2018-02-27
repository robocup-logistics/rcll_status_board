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
