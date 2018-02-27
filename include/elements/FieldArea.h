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
        void setWalls(std::vector<double> wall_coordinates);
        void setTeam(std::string team_name, rcll_draw::Team team_color);
        void draw(cv::Mat &mat);

    private:
        int x, y = 0;
        int w, h = 1;

        HStatusPanel game_info;
        TeamHeaderPanel team_cyan;
        TeamHeaderPanel team_magenta;
        GameField gamefield;

    };
}

#endif
