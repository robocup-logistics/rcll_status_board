#ifndef RCLL_TEAM_HEADER_PANEL_H
#define RCLL_TEAM_HEADER_PANEL_H

#include <util.h>
#include <BoxLabel.h>

namespace rcll_draw {

    class TeamHeaderPanel {
    public:
        TeamHeaderPanel();
        ~TeamHeaderPanel();

        void setTeam(std::string team_name, rcll_draw::Team team_color);
        void setGeometry(int x, int y, int w, int h);
        void draw(cv::Mat &mat);
    private:
        BoxLabel blbl_header_color;
        BoxLabel blbl_header_name;
    };
}

#endif
