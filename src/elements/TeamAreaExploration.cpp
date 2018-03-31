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
