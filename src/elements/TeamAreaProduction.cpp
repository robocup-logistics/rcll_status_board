#include <TeamAreaProduction.h>

// TeamAreaProduction ####################################################################

rcll_draw::TeamAreaProduction::TeamAreaProduction(){
    game_info = VStatusPanel(rcll_draw::NO_TEAM);
    product_info = ProductInfo();
    machine_info = MachineInfoProduction(rcll_draw::NO_TEAM);
    robot_info = RobotInfo(rcll_draw::NO_TEAM);
}

rcll_draw::TeamAreaProduction::TeamAreaProduction(rcll_draw::Team team){
    game_info = VStatusPanel(team);
    product_info = ProductInfo();
    machine_info = MachineInfoProduction(team);
    robot_info = RobotInfo(team);
}

rcll_draw::TeamAreaProduction::~TeamAreaProduction(){

}

void rcll_draw::TeamAreaProduction::setGeometry(int x, int y, int w, int h, int gapsize){
    int w1 = (w - gapsize);
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
    game_info.setGeometry(x, y, w1 * 0.12, h * 0.6);
    product_info.setGeometry(x + gapsize + w1 * 0.12, y, w1 * 0.88, h * 0.6, gapsize);
    machine_info.setGeometry(x, y + h * 0.6, w1 * 0.55, h * 0.4, gapsize);
    robot_info.setGeometry(x + w1 * 0.55 + gapsize, y + h * 0.6, w1 * 0.45, h * 0.4, gapsize);
}

void rcll_draw::TeamAreaProduction::setGameInfo(std::string gamestate, std::string gamephase, int time, int score){
    game_info.setContent(gamestate, gamephase, time, score);
}

void rcll_draw::TeamAreaProduction::setMachineName(std::string name_long, std::string name_short, int index){
    return machine_info.setMachineName(name_long, name_short, index);
}

void rcll_draw::TeamAreaProduction::setMachineStatus(std::string status, int index){
    machine_info.setMachineStatus(status, index);
}

void rcll_draw::TeamAreaProduction::setRobotName(std::string name, bool active, int index){
    return robot_info.setRobotName(name, active, index);
}

void rcll_draw::TeamAreaProduction::setRobotStatus(std::string activity, double active_time, int maintenance_count, int maintenance_max, int index){
    robot_info.setRobotStatus(activity, active_time, maintenance_count, maintenance_max, index);
}

void rcll_draw::TeamAreaProduction::setProduct(int id, Product plan, double progress, int deadline, int points, int points_max, int index){
    product_info.setProduct(id, plan, progress, deadline, points, points_max, index);
}

void rcll_draw::TeamAreaProduction::draw(cv::Mat &mat){
    game_info.draw(mat);
    product_info.draw(mat);
    machine_info.draw(mat);
    robot_info.draw(mat);
}
