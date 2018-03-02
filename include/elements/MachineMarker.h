#ifndef RCLL_MACHINE_MARKER_H
#define RCLL_MACHINE_MARKER_H

#include <util.h>
#include <BoxLabel.h>

namespace rcll_draw {
    class MachineMarker {
    public:
        MachineMarker();
        MachineMarker(Team team);
        ~MachineMarker();

        void draw(cv::Mat &mat);

    private:
        Team team;
        BoxLabel blbl_header;
    };
}

#endif
