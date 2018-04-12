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
        void setProduct(ProductInformation pi, int index);
        void setProductsCount(size_t count);
        void paging();
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
