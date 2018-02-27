#include <GameField.h>
// GameField ####################################################################

rcll_draw::GameField::GameField(){
    gamephase = rcll_draw::PRE_GAME;

    background.setBackgroundColor(rcll_draw::C_GREY_DARK);
    background.setBorderColor(rcll_draw::C_BLACK);
    background2.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    background2.setBorderColor(rcll_draw::C_BLACK);
    background2.setBorderSize(2);
}

rcll_draw::GameField::~GameField(){

}

void rcll_draw::GameField::setPhase(rcll_draw::GamePhase gamephase){
    this->gamephase = gamephase;
}

void rcll_draw::GameField::setLayout(double field_w, double field_h, int zones_x, int zones_y){
    // get the optimal field size
    int gapsize = h0 * 0.02;
    int pixel_per_meter = (h0 - 2 * gapsize) / field_h;
    h0 = (int)field_h * pixel_per_meter;
    w0 = (int)field_w * pixel_per_meter;
    x0 = x0;
    y0 = y0 - gapsize;

    int diff = w0 / (zones_x);
    for (int i = 0; i < zones_x + 1; i++){
        Line line;
        int x_line = x0 - (zones_x / 2) * diff + i * diff;
        line.setLine(x_line, y0 - h0, x_line, y0);
        line.setBorderSize(1);
        line.setBorderColor(rcll_draw::C_BLACK);
        zone_lines.push_back(line);
    }

    diff = h0 / (zones_y);
    for (int i = 0; i < zones_y + 1; i++){
        Line line;
        int y_line = y0 - h0 + i * diff;
        line.setLine(x0 - w0 / 2, y_line, x0 + w0 / 2, y_line);
        line.setBorderSize(1);
        line.setBorderColor(rcll_draw::C_BLACK);
        zone_lines.push_back(line);
    }

    background.setPos(x0 - w0 / 2 - gapsize, y0 - h0 - gapsize);
    background.setSize(w0 + 2 * gapsize, h0 + 2 * gapsize);
    background2.setPos(x0 - w0 /2, y0 - h0);
    background2.setSize(w0, h0);
}

void rcll_draw::GameField::setGeometry(int x, int y, int w, int h){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;

    w0 = w;
    h0 = h;
    x0 = x + w / 2;
    y0 = y + h;
}

void rcll_draw::GameField::draw(cv::Mat &mat){
    background.draw(mat);
    background2.draw(mat);

    for (size_t i = 0; i < zone_lines.size(); i++){
        zone_lines[i].draw(mat);
    }
}
