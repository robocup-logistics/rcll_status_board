#ifndef RCLL_GAMEFIELD_H
#define RCLL_GAMEFIELD_H

#include <util.h>
#include <Rectangle.h>
#include <BoxLabel.h>
#include <Line.h>

namespace rcll_draw {

    // ##################################################

    class GameField {
    public:
        GameField();
        ~GameField();

        void setPhase(rcll_draw::GamePhase gamephase);
        void setGeometry(int x, int y, int w, int h);
        void setLayout(double field_w, double field_h, int zones_x, int zones_y, std::vector<int> insertion_zones);
        void addWall(double x1, double y1, double x2, double y2);
        void draw(cv::Mat &mat);

    private:
        int x, y = 0;
        int w, h = 1;

        int x0, y0 = 0;
        int w0, h0 = 1;

        int pixel_per_meter = 10;

        GamePhase gamephase;
        Rectangle background;
        Rectangle background2;
        BoxLabel insertion_cyan1;
        BoxLabel insertion_cyan2;
        BoxLabel insertion_magenta1;
        BoxLabel insertion_magenta2;
        std::vector<Line> walls;
        std::vector<Line> zone_lines;
        std::vector<BoxLabel> zone_names;

    };
}

#endif
