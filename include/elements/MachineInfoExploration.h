#ifndef RCLL_MACHINE_INFO_EXPLORATION_H
#define RCLL_MACHINE_INFO_EXPLORATION_H

#include <util.h>
#include <BoxLabel.h>
#include <MachineLabelExploration.h>

namespace rcll_draw {

    class MachineInfoExploration {
    public:
        MachineInfoExploration();
        MachineInfoExploration(Team team);
        ~MachineInfoExploration();

        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setMachineName(std::string name_long, std::string name_short, int index);
        void setMachineStatus(int status1, int status2, int index);
        void draw(cv::Mat &mat);

    private:
        Team team;
        BoxLabel blbl_header1;
        BoxLabel blbl_header2;
        std::vector<MachineLabelExploration> machines;
    };
}

#endif
