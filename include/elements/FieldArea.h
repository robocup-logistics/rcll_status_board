/*
The MIT License (MIT)

Copyright (c) 2017-2018 Florian Eith <florian.eith@web.de>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

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
