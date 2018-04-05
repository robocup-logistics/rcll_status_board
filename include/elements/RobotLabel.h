#ifndef RCLL_ROBOT_LABEL_H
#define RCLL_ROBOT_LABEL_H

#include <util.h>
#include <BoxLabel.h>
#include <MultilineBoxLabel.h>

namespace rcll_draw {
    class RobotLabel {
    public:
        RobotLabel();
        ~RobotLabel();

        void setGeometry(int x, int y, int w, int h);
        void setRobotName(int id, std::string name, bool active);
        void setRobotStatus(std::string mlblbl_activity, double active_time, int blbl_maintenance, int maintenance_max);
        void draw(cv::Mat &mat);

    private:
        bool active;

        BoxLabel blbl_name;
        MultilineBoxLabel mlblbl_activity;
        BoxLabel blbl_activetime;
        BoxLabel blbl_maintenance;
    };
}

#endif
