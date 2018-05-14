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

#ifndef RCLL_MACHINE_LABEL_PRODUCTION_H
#define RCLL_MACHINE_LABEL_PRODUCTION_H

#include <util.h>
#include <BoxLabel.h>

#include <rcll_vis_msgs/Machine.h>

namespace rcll_draw {
    class MachineLabelProduction {
    public:
        MachineLabelProduction(rcll_draw::Team team);
        ~MachineLabelProduction();

        void setGeometry(int x, int y, int w, int h);
        void setMachine(rcll_vis_msgs::Machine &machine);
        void draw(cv::Mat &mat);

    private:
        BoxLabel blbl_border;
        BoxLabel blbl_machinename;
        Color lamp1_color;
        Color lamp2_color;
        BoxLabel blbl_lamp1;
        BoxLabel blbl_lamp2;
        bool flashing;
    };
}

#endif
