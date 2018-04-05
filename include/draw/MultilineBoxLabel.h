#ifndef RCLL_MULTILINEBOXLABEL_H
#define RCLL_MULTILINEBOXLABEL_H

#include <util.h>
#include <BoxLabel.h>

namespace rcll_draw {
    class MultilineBoxLabel : public BoxLabel {
    public:
        MultilineBoxLabel();
        ~MultilineBoxLabel();

        void setLines(int lines);

        void draw(cv::Mat &mat);

        int lines = 1;
    };
}

#endif
