#ifndef RCLL_HSTATUS_PANEL_H
#define RCLL_HSTATUS_PANEL_H

#include <util.h>
#include <BoxLabel.h>

namespace rcll_draw {

    class HStatusPanel {
    public:
        HStatusPanel();
        ~HStatusPanel();

        void setGeometry(int x, int y, int w, int h);
        void setContent(std::string gamestate, std::string gamephase, int time, int score_cyan, int score_magenta);
        void draw(cv::Mat &mat);
    private:
        BoxLabel blbl_state_header;
        BoxLabel blbl_state_value;
        BoxLabel blbl_phase_header;
        BoxLabel blbl_phase_value;
        BoxLabel blbl_time_header;
        BoxLabel blbl_time_value;
        BoxLabel blbl_score_header;
        BoxLabel blbl_score_value_cyan;
        BoxLabel blbl_score_value_mid;
        BoxLabel blbl_score_value_magenta;
    };
}

#endif
