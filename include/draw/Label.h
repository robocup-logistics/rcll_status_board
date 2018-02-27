#ifndef RCLL_LABEL_H
#define RCLL_LABEL_H

#include <util.h>

namespace rcll_draw {
    class Label {
    public:
        Label();
        ~Label();

        void setContent(std::string content);
        void setPos(int x, int y); // origin is top left corner
        void setFrontColor(Color c);
        void setFont(std::string font, double fontsize);
        void setFontSize(double fontsize);
        void setAlignment(Alignment alignment);
        void draw(cv::Mat &mat);

    protected:
        int x, y = 0;
        std::string content = "";
        Color frontcolor = C_BLACK;
        std::string font;
        double fontsize = 12.0;
        Alignment alignment = CenterCenter;
    };
}

#endif
