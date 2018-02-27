#ifndef RCLL_ROBOT_INFO_H
#define RCLL_ROBOT_INFO_H

#include <util.h>
#include <BoxLabel.h>
#include <RobotLabel.h>

namespace rcll_draw {
    class RobotInfo {
    public:
        RobotInfo();
        RobotInfo(Team team);
        ~RobotInfo();

        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setRobotName(std::string name_str, bool active, int index);
        void setRobotStatus(std::string activity, double active_time, int maintenance_count, int maintenance_max, int index);
        void draw(cv::Mat &mat);

    private:
        Team team;
        BoxLabel blbl_header;
        std::vector<RobotLabel> robots;
    };
}

#endif
