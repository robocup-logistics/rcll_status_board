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

#include <FieldArea.h>
// FieldArea ####################################################################

rcll_draw::FieldArea::FieldArea(){
    game_info = HStatusPanel();
    team_cyan = TeamHeaderPanel();
    team_magenta = TeamHeaderPanel();
    gamefield = GameField();

    blbl_text.setContent("first icon: reported position    second icon: reported orientation");
    blbl_text.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_text.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_text.setBorderColor(rcll_draw::C_WHITE);
    blbl_text.setFontSize(1.0);
}

rcll_draw::FieldArea::~FieldArea(){

}

void rcll_draw::FieldArea::setGeometry(int x, int y, int w, int h, int gapsize){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
    game_info.setGeometry(x + w * 0.2, y, w * 0.6, h * 0.1);
    team_cyan.setGeometry(x, y, w * 0.2, h * 0.1);
    team_magenta.setGeometry(x + w * 0.8, y, w * 0.2, h * 0.1);
    gamefield.setGeometry(x, y + h * 0.1 + gapsize, w, h * 0.9 - gapsize);
    blbl_text.setSize(w, h * 0.1);
    blbl_text.setPos(x, y + h - gapsize);
}

void rcll_draw::FieldArea::setGameInfo(std::string gamestate, std::string gamephase, int time, int score_cyan, int score_magenta){
    game_info.setContent(gamestate, gamephase, time, score_cyan, score_magenta);
    if (gamephase == "PRE GAME"){
        gamefield.setPhase(rcll_draw::PRE_GAME);
        this->gamephase = rcll_draw::PRE_GAME;
    } else if (gamephase == "SETUP"){
        gamefield.setPhase(rcll_draw::SETUP);
        this->gamephase = rcll_draw::SETUP;
    } else if (gamephase == "EXPLORATION"){
        gamefield.setPhase(rcll_draw::EXPLORATION);
        this->gamephase = rcll_draw::EXPLORATION;
    } else if (gamephase == "PRODUCTION"){
        gamefield.setPhase(rcll_draw::PRODUCTION);
        this->gamephase = rcll_draw::PRODUCTION;
    } else if (gamephase == "POST GAME"){
        gamefield.setPhase(rcll_draw::POST_GAME);
        this->gamephase = rcll_draw::POST_GAME;
    } else {
        gamefield.setPhase(rcll_draw::PRE_GAME);
        this->gamephase = rcll_draw::PRE_GAME;
    }
}

void rcll_draw::FieldArea::setLayout(double field_w, double field_h, int zones_x, int zones_y, std::vector<int> insertion_zones){
    gamefield.setLayout(field_w, field_h, zones_x, zones_y, insertion_zones);
}

void rcll_draw::FieldArea::setWalls(std::vector<float> wall_coordinates){
    for (size_t i = 0; i < wall_coordinates.size(); i+=4){
        if (i + 3 < wall_coordinates.size()){
            gamefield.addWall(wall_coordinates[i], wall_coordinates[i + 1], wall_coordinates[i + 2], wall_coordinates[i + 3]);
        }
    }
}

void rcll_draw::FieldArea::setTeam(std::string team_name, rcll_draw::Team team_color){
    if (team_color == rcll_draw::CYAN){
        team_cyan.setTeam(team_name, team_color);
    } else if (team_color == rcll_draw::MAGENTA){
        team_magenta.setTeam(team_name, team_color);
    } else {
        ROS_WARN("Invalid team color");
    }
}

size_t rcll_draw::FieldArea::addRobot(std::string name, int id, rcll_draw::Team team){
    return gamefield.addRobot(name, id, team);
}

void rcll_draw::FieldArea::setRobotPos(double x, double y, double yaw, size_t index){
    gamefield.setRobotPos(x, y, yaw, index);
}

void rcll_draw::FieldArea::setMachine(std::string name, rcll_draw::Team team, size_t index){
    return gamefield.setMachine(name, team, index);
}

void rcll_draw::FieldArea::setMachinePos(double x, double y, double yaw, size_t index){
    gamefield.setMachinePos(x, y, yaw, index);
}

void rcll_draw::FieldArea::setMachineReport(int report1_status, int report2_status, size_t index){
    gamefield.setMachineReport(report1_status, report2_status, index);
}

void rcll_draw::FieldArea::draw(cv::Mat &mat){
    game_info.draw(mat);
    team_cyan.draw(mat);
    team_magenta.draw(mat);
    gamefield.draw(mat);

    if (gamephase == rcll_draw::EXPLORATION){
        blbl_text.draw(mat);
    }
}
