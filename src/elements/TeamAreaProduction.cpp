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

#include <TeamAreaProduction.h>

// TeamAreaProduction ####################################################################

rcll_draw::TeamAreaProduction::TeamAreaProduction(){
    vsp_gameinfo = VStatusPanel(rcll_draw::NO_TEAM);
    pi_productinfo = ProductInfo();
    mip_machineinfo = MachineInfoProduction(rcll_draw::NO_TEAM);
    ri_robotinfo = RobotInfo(rcll_draw::NO_TEAM);
}

rcll_draw::TeamAreaProduction::TeamAreaProduction(rcll_draw::Team team){
    vsp_gameinfo = VStatusPanel(team);
    pi_productinfo = ProductInfo();
    mip_machineinfo = MachineInfoProduction(team);
    ri_robotinfo = RobotInfo(team);
}

rcll_draw::TeamAreaProduction::~TeamAreaProduction(){

}

void rcll_draw::TeamAreaProduction::setGeometry(int x, int y, int w, int h, int gapsize){
    int w1 = (w - gapsize);
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
    vsp_gameinfo.setGeometry(x, y, w1 * 0.12, h * 0.6);
    pi_productinfo.setGeometry(x + gapsize + w1 * 0.12, y, w1 * 0.88, h * 0.6, gapsize);
    mip_machineinfo.setGeometry(x, y + h * 0.6, w1 * 0.55, h * 0.4, gapsize);
    ri_robotinfo.setGeometry(x + w1 * 0.55 + gapsize, y + h * 0.6, w1 * 0.45, h * 0.4, gapsize);
}

void rcll_draw::TeamAreaProduction::setGameInfo(rcll_vis_msgs::GameInfo &gameinfo){
    vsp_gameinfo.setContent(gameinfo);
}

void rcll_draw::TeamAreaProduction::setMachines(std::vector<rcll_vis_msgs::Machine> &machines){
    return mip_machineinfo.setMachines(machines);
}

void rcll_draw::TeamAreaProduction::setRobots(std::vector<rcll_vis_msgs::Robot> &robots){
    return ri_robotinfo.setRobots(robots);
}

void rcll_draw::TeamAreaProduction::setProduct(ProductInformation pi, int index){
    pi_productinfo.setProduct(pi, index);
}

void rcll_draw::TeamAreaProduction::setProductsCount(size_t count){
    pi_productinfo.setProductsCount(count);
}

void rcll_draw::TeamAreaProduction::paging(){
    pi_productinfo.paging();
}

void rcll_draw::TeamAreaProduction::draw(cv::Mat &mat){
    vsp_gameinfo.draw(mat);
    pi_productinfo.draw(mat);
    mip_machineinfo.draw(mat);
    ri_robotinfo.draw(mat);
}
