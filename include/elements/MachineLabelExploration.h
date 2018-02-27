#ifndef RCLL_MACHINE_LABEL_EXPLORATION_H
#define RCLL_MACHINE_LABEL_EXPLORATION_H

#include <util.h>
#include <BoxLabel.h>
#include <Image.h>

namespace rcll_draw {
    class MachineLabelExploration {
    public:
        MachineLabelExploration();
        ~MachineLabelExploration();

        void setGeometry(int x, int y, int w, int h);
        void setMachineName(std::string name, int index);
        void setMachineStatus(int status1, int status2);
        void setHeader(std::string status1, std::string status2);
        void draw(cv::Mat &mat);

    private:
        BoxLabel blbl_machinename;
        BoxLabel blbl_status1;
        BoxLabel blbl_status2;
        Image img_status1;
        Image img_status2;
    };
}

#endif
