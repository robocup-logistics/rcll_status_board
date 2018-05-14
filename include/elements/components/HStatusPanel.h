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

#ifndef RCLL_HSTATUS_PANEL_H
#define RCLL_HSTATUS_PANEL_H

#include <util.h>
#include <BoxLabel.h>

#include <rcll_vis_msgs/GameInfo.h>

namespace rcll_draw {

    class HStatusPanel {
    public:
        HStatusPanel();
        ~HStatusPanel();

        void setGeometry(int x, int y, double scale);

        int getW(double scale = 1.0);
        int getH(double scale = 1.0);

        void setContent(rcll_vis_msgs::GameInfo &gameinfo);
        void draw(cv::Mat &mat, bool show_element_border = false);
    private:
        int x, y = 0;
        int w = 1087, h = 86;
        double scale = 1.0;
        cv::Mat origin;

        BoxLabel blbl_state_header;
        BoxLabel blbl_state_value;
        BoxLabel blbl_phase_header;
        BoxLabel blbl_phase_value;
        BoxLabel blbl_time_header;
        BoxLabel blbl_time_value;
        BoxLabel blbl_score_header;
        BoxLabel blbl_score_value_cyan;
        BoxLabel blbl_score_value_mid;
        BoxLabel blbl_score_value_magenta;
    };
}

#endif
