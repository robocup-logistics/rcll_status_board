#ifndef RCLL_MACHINE_INFO_PRODUCTION_H
#define RCLL_MACHINE_INFO_PRODUCTION_H

#include <util.h>
#include <BoxLabel.h>
#include <MachineLabelProduction.h>

namespace rcll_draw {

    class MachineInfoProduction {
    public:
        MachineInfoProduction();
        MachineInfoProduction(Team team);
        ~MachineInfoProduction();

        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setMachineName(std::string name_long, std::string name_short, int index);
        void setMachineStatus(std::string status, int index);
        void draw(cv::Mat &mat);

    private:
        Team team;
        BoxLabel blbl_header;
        std::vector<MachineLabelProduction> machines;
    };
}

#endif
