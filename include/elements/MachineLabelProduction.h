#ifndef RCLL_MACHINE_LABEL_PRODUCTION_H
#define RCLL_MACHINE_LABEL_PRODUCTION_H

#include <util.h>
#include <BoxLabel.h>

namespace rcll_draw {
    class MachineLabelProduction {
    public:
        MachineLabelProduction();
        ~MachineLabelProduction();

        void setGeometry(int x, int y, int w, int h);
        void setMachineName(std::string name);
        void setMachineStatus(std::string status, Color lamp_top, Color lamp_bottom, std::string lamp_top_str, std::string lamp_bottom_str);
        void setFlashing(bool flashing);
        void draw(cv::Mat &mat);

    private:
        BoxLabel blbl_border;
        BoxLabel blbl_machinename;
        BoxLabel blbl_state;
        Color lamp1_color;
        Color lamp2_color;
        BoxLabel blbl_lamp1;
        BoxLabel blbl_lamp2;
        bool flashing;
    };
}

#endif
