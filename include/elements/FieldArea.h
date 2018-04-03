#ifndef RCLL_FIELD_AREA_H
#define RCLL_FIELD_AREA_H

#include <util.h>
#include <HStatusPanel.h>
#include <TeamHeaderPanel.h>
#include <GameField.h>

namespace rcll_draw {

    // ##################################################

    class FieldArea {
    public:
        FieldArea();
        ~FieldArea();

        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setGameInfo(std::string gamestate, std::string gamephase, int time, int score_cyan, int score_magenta);
        void setLayout(double field_w, double field_h, int zones_x, int zones_y, std::vector<int> insertion_zones);
        void setWalls(std::vector<float> wall_coordinates);
        void setTeam(std::string team_name, rcll_draw::Team team_color);
        size_t addRobot(std::string name, int id, rcll_draw::Team team);
        void setRobotPos(double x, double y, double yaw, size_t index);
        void setMachine(std::string name, rcll_draw::Team team, size_t index);
        void setMachinePos(double x, double y, double yaw, size_t index);
        void setMachineReport(int report1_status, int report2_status, size_t index);
        void draw(cv::Mat &mat);

    private:
        int x, y = 0;
        int w, h = 1;

        rcll_draw::GamePhase gamephase;

        HStatusPanel game_info;
        TeamHeaderPanel team_cyan;
        TeamHeaderPanel team_magenta;
        GameField gamefield;
        BoxLabel blbl_text;

    };
}

#endif
