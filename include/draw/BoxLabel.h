#ifndef RCLL_BOXLABEL_H
#define RCLL_BOXLABEL_H

#include <util.h>
#include <Label.h>

namespace rcll_draw {
    class BoxLabel : public Label {
    public:
        BoxLabel();
        ~BoxLabel();

        void setSize(int w, int h);
        void setBackgroundColor(Color c);
        void setBorderColor(Color c);
        void setBorderSize(int bordersize);
        void draw(cv::Mat &mat);

    protected:
        int w, h = 10;
        Color bordercolor = C_BLACK;
        Color backgroundcolor = C_WHITE;
        double angle = 0;
        int bordersize = 1;

    };
}

#endif
