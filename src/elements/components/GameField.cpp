/*
The MIT License (MIT)

Copyright (c) 2017-2018 Florian Eith <florian.eith@web.de>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <GameField.h>
// GameField ####################################################################

rcll_draw::GameField::GameField(){
    origin = cv::Mat(h, w, CV_8UC4);
    gamephase = rcll_draw::PRE_GAME;

    rct_background.setBackgroundColor(rcll_draw::C_GREY_DARK);
    rct_background.setBorderColor(rcll_draw::C_BLACK);

    rct_background2.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    rct_background2.setBorderColor(rcll_draw::C_GREY_LIGHT);
    rct_background2.setBorderSize(1);

    blbl_insertion_cyan1.setContent("Insertion");
    blbl_insertion_cyan1.setAlignment(rcll_draw::CenterCenter);
    blbl_insertion_cyan1.setBorderColor(rcll_draw::C_CYAN_DARK);
    blbl_insertion_cyan1.setBackgroundColor(rcll_draw::C_CYAN_DARK);
    blbl_insertion_cyan1.setFrontColor(rcll_draw::C_WHITE);
    blbl_insertion_cyan1.setFontSize(0.75);

    blbl_insertion_cyan2.setContent("Zone");
    blbl_insertion_cyan2.setAlignment(rcll_draw::CenterCenter);
    blbl_insertion_cyan2.setBorderColor(rcll_draw::C_CYAN_DARK);
    blbl_insertion_cyan2.setBackgroundColor(rcll_draw::C_CYAN_DARK);
    blbl_insertion_cyan2.setFrontColor(rcll_draw::C_WHITE);
    blbl_insertion_cyan2.setFontSize(0.75);

    blbl_insertion_magenta1.setContent("Insertion");
    blbl_insertion_magenta1.setAlignment(rcll_draw::CenterCenter);
    blbl_insertion_magenta1.setBorderColor(rcll_draw::C_MAGENTA_DARK);
    blbl_insertion_magenta1.setBackgroundColor(rcll_draw::C_MAGENTA_DARK);
    blbl_insertion_magenta1.setFrontColor(rcll_draw::C_WHITE);
    blbl_insertion_magenta1.setFontSize(0.75);

    blbl_insertion_magenta2.setContent("Zone");
    blbl_insertion_magenta2.setAlignment(rcll_draw::CenterCenter);
    blbl_insertion_magenta2.setBorderColor(rcll_draw::C_MAGENTA_DARK);
    blbl_insertion_magenta2.setBackgroundColor(rcll_draw::C_MAGENTA_DARK);
    blbl_insertion_magenta2.setFrontColor(rcll_draw::C_WHITE);
    blbl_insertion_magenta2.setFontSize(0.75);

    mm_machine_markers.resize(14);
}

rcll_draw::GameField::~GameField(){

}

void rcll_draw::GameField::setPhase(rcll_draw::GamePhase gamephase){
    this->gamephase = gamephase;
}


void rcll_draw::GameField::setGeometry(int x, int y, double scale){
    this->x = x;
    this->y = y;
    this->scale = scale;
}

int rcll_draw::GameField::getW(double scale){
    return (int)((double)w * scale);
}

int rcll_draw::GameField::getH(double scale){
    return (int)((double)h * scale);
}

void rcll_draw::GameField::setRefBoxView(bool refbox_view){
    this->refbox_view = refbox_view;
}

void rcll_draw::GameField::setGameField(rcll_vis_msgs::SetGameField &setgamefield){
    int refbox_factor = 1.0;
    int zone_offset_px, zone_offset_nx, zone_offset_y = 0;

    // get the optimal field size
    int gapsize = h * 0.02;
    pixel_per_meter = (h - 2 * gapsize) / setgamefield.field_width;
    int h0 = (int)setgamefield.field_width * pixel_per_meter;
    int w0 = (int)setgamefield.field_length * pixel_per_meter;


    if (refbox_view){
        refbox_factor = -1;
        origin_x = gapsize + w0 / 2;
        origin_y = gapsize + h0;
        zone_offset_px = 0;
        zone_offset_nx = 1;
        zone_offset_y = -1;
    } else {
        refbox_factor = 1;
        origin_x = gapsize + w0 / 2;
        origin_y = gapsize;
        zone_offset_px = -1;
        zone_offset_nx = 0;
        zone_offset_y = 0;
    }

    // add walls
    for (size_t i = 0; i < setgamefield.walls.size(); i+=4){
        Line line;
        line.setLineByPoints(origin_x + refbox_factor * setgamefield.walls[i] * pixel_per_meter, origin_y + refbox_factor * setgamefield.walls[i+1] * pixel_per_meter, origin_x + refbox_factor * setgamefield.walls[i+2] * pixel_per_meter, origin_y + refbox_factor * setgamefield.walls[i+3] * pixel_per_meter);
        line.setBorderSize(4);
        line.setBorderColor(rcll_draw::C_BLACK);
        lnes_walls.push_back(line);
    }

    // calculate the vertical grid lines
    int zone_w = w0 / (setgamefield.zones_x);
    for (int i = 0; i < setgamefield.zones_x + 1; i++){
        Line line;
        int x_line = gapsize + i * zone_w;
        line.setLineByPoints(x_line, origin_y, x_line, origin_y + refbox_factor * h0);
        line.setBorderSize(1);
        line.setBorderColor(rcll_draw::C_BLACK);
        lnes_zone_lines.push_back(line);
    }

    // calculate the horizontal grid lines
    int zone_h = h0 / (setgamefield.zones_y);
    for (int i = 0; i < setgamefield.zones_y + 1; i++){
        Line line;
        int y_line = gapsize + i * zone_h;
        line.setLineByPoints(origin_x + w0 / 2, y_line, origin_x - w0 / 2, y_line);
        line.setBorderSize(1);
        line.setBorderColor(rcll_draw::C_BLACK);
        lnes_zone_lines.push_back(line);
    }

    // calculate the zone labels
    for (int zx = -setgamefield.zones_x/2; zx <= setgamefield.zones_x/2; zx++){
        if (zx == 0){
            continue;
        }
        for (int zy = 1; zy <= setgamefield.zones_y; zy++){
            BoxLabel zone_label;
            int unsigned_zone_number = 10 * abs(zx) + zy;

            bool cancel = false;
            for (size_t i = 0; i < setgamefield.insertion_zones.size(); i++){
                if (setgamefield.insertion_zones[i] == unsigned_zone_number){
                    cancel = true;
                    break;
                }
            }

            if (cancel){
                continue;
            }

            if (zx < 0){
                zone_label.setContent("C-Z" + std::to_string(unsigned_zone_number) + " ");
                zone_label.setAlignment(rcll_draw::CenterCenter);
                zone_label.setPos(origin_x + refbox_factor * (zx + zone_offset_nx) * zone_w, origin_y + refbox_factor * ((zy + zone_offset_y) * zone_h) - zone_h / 4);
            } else {
                zone_label.setContent(" M-Z" + std::to_string(unsigned_zone_number));
                zone_label.setAlignment(rcll_draw::CenterCenter);
                zone_label.setPos(origin_x + refbox_factor * (zx + zone_offset_px) * zone_w, origin_y + refbox_factor * ((zy + zone_offset_y) * zone_h) - zone_h / 4);
            }
            zone_label.setBorderColor(rcll_draw::C_TRANSPARENT);
            zone_label.setBackgroundColor(rcll_draw::C_TRANSPARENT);
            zone_label.setFrontColor(rcll_draw::C_BLACK);
            zone_label.setBorderSize(1);
            zone_label.setFontSize(0.5);

            zone_label.setSize(zone_w, zone_h / 4);
            blbls_zone_names.push_back(zone_label);
        }
    }

    rct_background.setPos(0, 0);
    rct_background.setSize(w0 + 2 * gapsize, h0 + 2 * gapsize);
    rct_background2.setPos(0, 0);
    rct_background2.setSize(w0 + 2 * gapsize, h0 + 2 * gapsize);

    int h1 = (zone_h - 2 * gapsize) / 2;
    blbl_insertion_cyan1.setSize(3 * zone_w - 2 * gapsize, h1);
    blbl_insertion_cyan2.setSize(3 * zone_w - 2 * gapsize, h1);
    blbl_insertion_magenta1.setSize(3 * zone_w - 2 * gapsize, h1);
    blbl_insertion_magenta2.setSize(3 * zone_w - 2 * gapsize, h1);

    blbl_insertion_cyan1.setPos(origin_x - refbox_factor * (setgamefield.zones_x/2 - zone_offset_nx * 3) * zone_w + gapsize, origin_y + refbox_factor * zone_h / 2 - h1);
    blbl_insertion_cyan2.setPos(origin_x - refbox_factor * (setgamefield.zones_x/2 - zone_offset_nx * 3) * zone_w + gapsize, origin_y + refbox_factor * zone_h / 2);
    blbl_insertion_magenta1.setPos(origin_x + refbox_factor * (setgamefield.zones_x/2 + zone_offset_px * 3) * zone_w + gapsize, origin_y + refbox_factor * zone_h / 2 - h1);
    blbl_insertion_magenta2.setPos(origin_x + refbox_factor * (setgamefield.zones_x/2 + zone_offset_px * 3) * zone_w + gapsize, origin_y + refbox_factor * zone_h / 2);
}

void rcll_draw::GameField::setRobots(std::vector<rcll_vis_msgs::Robot> &robots){
    rm_robot_markers.clear();
    for (size_t i = 0; i < robots.size(); i++){
        RobotMarker r((rcll_draw::Team)robots[i].team);
        r.setOrigin(origin_x, origin_y, pixel_per_meter);
        r.setRefBoxView(refbox_view);
        r.setRobot(robots[i]);
        rm_robot_markers.push_back(r);
    }
}

void rcll_draw::GameField::setMachines(std::vector<rcll_vis_msgs::Machine> &machines){
    mm_machine_markers.clear();
    for (size_t i = 0; i < machines.size(); i++){
        MachineMarker m((rcll_draw::Team)machines[i].team);
        m.setOrigin(origin_x, origin_y, pixel_per_meter);
        m.setRefBoxView(refbox_view);
        m.setMachine(machines[i]);
        m.setPhase(gamephase);
        mm_machine_markers.push_back(m);
    }
}

void rcll_draw::GameField::draw(cv::Mat &mat, bool show_element_border){
    cv::rectangle(origin, cv::Point(0, 0), cv::Point (w-1, h-1), rcll_draw::getColor(rcll_draw::C_WHITE), cv::FILLED);

    rct_background2.draw(origin);

    for (size_t i = 0; i < lnes_walls.size(); i++){
        lnes_walls[i].draw(origin);
    }

    if (gamephase == rcll_draw::SETUP){
        for (size_t i = 0; i < lnes_zone_lines.size(); i++){
            lnes_zone_lines[i].draw(origin);
        }

        for (size_t i = 0; i < blbls_zone_names.size(); i++){
            blbls_zone_names[i].draw(origin);
        }
    }

    blbl_insertion_cyan1.draw(origin);
    blbl_insertion_cyan2.draw(origin);
    blbl_insertion_magenta1.draw(origin);
    blbl_insertion_magenta2.draw(origin);

    for (size_t i = 0; i < rm_robot_markers.size(); i++){
        rm_robot_markers[i].draw(origin);
    }

    for (size_t i = 0; i < mm_machine_markers.size(); i++){
        mm_machine_markers[i].draw(origin);
    }

    if (gamephase == rcll_draw::SETUP){
        if (refbox_view){
            cv::line(origin, cv::Point(origin_x, origin_y), cv::Point(origin_x + 30, origin_y), rcll_draw::getColor(rcll_draw::C_RED), 3, 8, 0);
            cv::line(origin, cv::Point(origin_x, origin_y), cv::Point(origin_x, origin_y - 30), rcll_draw::getColor(rcll_draw::C_GREEN_LIGHT), 3, 8, 0);
        } else {
            cv::line(origin, cv::Point(origin_x, origin_y), cv::Point(origin_x - 30, origin_y), rcll_draw::getColor(rcll_draw::C_RED), 3, 8, 0);
            cv::line(origin, cv::Point(origin_x, origin_y), cv::Point(origin_x, origin_y + 30), rcll_draw::getColor(rcll_draw::C_GREEN_LIGHT), 3, 8, 0);
        }
    }

    if (show_element_border){
        cv::rectangle(origin, cv::Point(0, 0), cv::Point (w-1, h-1), rcll_draw::getColor(rcll_draw::C_RED), 1);
    }

    rcll_draw::mergeImages(mat, origin, x, y, scale);
}
