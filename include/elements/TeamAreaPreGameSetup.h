#ifndef RCLL_TEAM_AREA_PREGAME_SETUP_H
#define RCLL_TEAM_AREA_PREGAME_SETUP_H

#include <util.h>
#include <TeamHeaderPanel.h>
#include <HStatusPanel.h>
#include <BoxLabel.h>

namespace rcll_draw {

    // ##################################################

    class TeamAreaPreGameSetup {
    public:
        TeamAreaPreGameSetup();
        TeamAreaPreGameSetup(Team team);
        ~TeamAreaPreGameSetup();

        void setTeams(std::string team_name_cyan, std::string team_name_magenta);
        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setGameInfo(std::string gamestate, std::string gamephase, int time, int score_cyan, int score_magenta);
        void draw(cv::Mat &mat);

    private:
        int x, y = 0;
        int w, h = 1;

        TeamHeaderPanel thpan_team_cyan;
        TeamHeaderPanel thpan_team_magenta;
        BoxLabel blbl_versus;
        BoxLabel tlbl_text;
        HStatusPanel game_info;
    };
}

#endif
