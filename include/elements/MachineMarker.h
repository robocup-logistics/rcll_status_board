#ifndef RCLL_MACHINE_MARKER_H
#define RCLL_MACHINE_MARKER_H

#include <util.h>
#include <BoxLabel.h>
#include <Line.h>
#include <Circle.h>

namespace rcll_draw {
    class MachineMarker {
    public:
        MachineMarker();
        MachineMarker(Team team);
        ~MachineMarker();

        void setPhase(rcll_draw::GamePhase gamephase);
        void setOrigin(int x0, int y0, int pixel_per_meter);
        void setMachineParams(std::string name, double w, double h);
        void setPos(double x, double y, double yaw);
        void recalculate();
        void draw(cv::Mat &mat);


    private:
        int xm, ym = 0;
        int pixel_per_meter = 10;
        int x0, y0 = 0;
        double w, h = 10;
        double yaw;

        cv::Mat img;

        GamePhase gamephase;
        Team team;
        BoxLabel blbl_machine;
        Line ln_input;
        Circle crc_output;
        BoxLabel blbl_in;
        BoxLabel blbl_out;
    };
}

#endif
