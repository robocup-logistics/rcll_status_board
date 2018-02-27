#ifndef RCLL_VSTATUS_PANEL_H
#define RCLL_VSTATUS_PANEL_H

#include <util.h>
#include <BoxLabel.h>

namespace rcll_draw {
    class VStatusPanel {
    public:
        VStatusPanel();
        VStatusPanel(Team team);
        ~VStatusPanel();

        void setGeometry(int x, int y, int w, int h);
        void setContent(std::string gamestate, std::string gamephase, int time, int score);
        void draw(cv::Mat &mat);
    private:
        Team team;
        BoxLabel blbl_state_header;
        BoxLabel blbl_state_value;
        BoxLabel blbl_phase_header;
        BoxLabel blbl_phase_value;
        BoxLabel blbl_time_header;
        BoxLabel blbl_time_value;
        BoxLabel blbl_score_header;
        BoxLabel blbl_score_value;
    };
}

#endif
