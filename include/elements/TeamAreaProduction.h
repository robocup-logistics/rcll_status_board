#ifndef RCLL_TEAM_AREA_PRODUCTION_H
#define RCLL_TEAM_AREA_PRODUCTION_H

#include <util.h>
#include <VStatusPanel.h>
#include <ProductInfo.h>
#include <MachineInfoProduction.h>
#include <RobotInfo.h>

namespace rcll_draw {
    class TeamAreaProduction {
    public:
        TeamAreaProduction();
        TeamAreaProduction(Team team);
        ~TeamAreaProduction();

        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setGameInfo(std::string gamestate, std::string gamephase, int time, int score);
        void setMachineName(std::string name_long, std::string name_short, int index);
        void setMachineStatus(std::string status, int index);
        void setRobotName(int id, std::string name, bool active, int index);
        void setRobotStatus(std::string activity, double active_time, int maintenance_count, int maintenance_max, int index);
        void setProduct(int id, Product plan, double progress, int deadline, int points, int points_max, int index);
        void draw(cv::Mat &mat);

    private:
        int x, y = 0;
        int w, h = 1;

        Team team;
        VStatusPanel game_info;
        ProductInfo product_info;
        MachineInfoProduction machine_info;
        RobotInfo robot_info;


    };
}

#endif
