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

#include <TeamAreaExploration.h>
// TeamAreaExploration ####################################################################

rcll_draw::TeamAreaExploration::TeamAreaExploration(){

}

rcll_draw::TeamAreaExploration::TeamAreaExploration(rcll_draw::Team team){
    game_info = HStatusPanel();
    machine_info = MachineInfoExploration(team);
}

rcll_draw::TeamAreaExploration::~TeamAreaExploration(){

}

void rcll_draw::TeamAreaExploration::setGeometry(int x, int y, int w, int h, int gapsize){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
    game_info.setGeometry(x + w * 0.2, y, w * 0.6, h * 0.1);
    machine_info.setGeometry(x + w * 0.2, y + h * 0.2, w * 0.6, h * 0.8, gapsize);
}

void rcll_draw::TeamAreaExploration::setGameInfo(std::string gamestate, std::string gamephase, int time, int score_cyan, int score_magenta){
    game_info.setContent(gamestate, gamephase, time, score_cyan, score_magenta);
}

void rcll_draw::TeamAreaExploration::setMachineName(std::string name_long, std::string name_short, int index){
    machine_info.setMachineName(name_long, name_short, index);
}

void rcll_draw::TeamAreaExploration::setMachineStatus(int status1, int status2, int index){
    machine_info.setMachineStatus(status1, status2, index);
}

void rcll_draw::TeamAreaExploration::draw(cv::Mat &mat){
    game_info.draw(mat);
    machine_info.draw(mat);
}
