#ifndef RCLL_TEAM_AREA_EXPLORATION_H
#define RCLL_TEAM_AREA_EXPLORATION_H

#include <util.h>
#include <HStatusPanel.h>
#include <MachineInfoExploration.h>

namespace rcll_draw {

    // ##################################################

    class TeamAreaExploration {
    public:
        TeamAreaExploration(Team team);
        ~TeamAreaExploration();

        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setGameInfo(std::string gamestate, std::string gamephase, int time, int score_cyan, int score_magenta);
        void setMachineName(std::string name, int index);
        void setMachineStatus(int status1, int status2, int index);
        void draw(cv::Mat &mat);

    private:
        int x, y = 0;
        int w, h = 1;

        Team team;
        HStatusPanel game_info;
        MachineInfoExploration machine_info;


    };
}

#endif
