#ifndef RCLL_HEADER_PANEL_H
#define RCLL_HEADER_PANEL_H

#include <util.h>
#include <BoxLabel.h>

namespace rcll_draw {

    class HeaderPanel {
    public:
        HeaderPanel(std::string content, Team team);
        ~HeaderPanel();

        void setGeometry(int y, int w, int h);
        void draw(cv::Mat &mat);
    private:
        BoxLabel blbl_header;
    };
}

#endif
