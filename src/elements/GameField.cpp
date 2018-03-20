#include <GameField.h>
// GameField ####################################################################

rcll_draw::GameField::GameField(){
    gamephase = rcll_draw::PRE_GAME;

    background.setBackgroundColor(rcll_draw::C_GREY_DARK);
    background.setBorderColor(rcll_draw::C_BLACK);

    background2.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    background2.setBorderColor(rcll_draw::C_GREY_LIGHT);
    background2.setBorderSize(1);

    insertion_cyan1.setContent("Insertion");
    insertion_cyan1.setAlignment(rcll_draw::CenterCenter);
    insertion_cyan1.setBorderColor(rcll_draw::C_CYAN_DARK);
    insertion_cyan1.setBackgroundColor(rcll_draw::C_CYAN_DARK);
    insertion_cyan1.setFrontColor(rcll_draw::C_BLACK);
    insertion_cyan1.setFontSize(0.75);

    insertion_cyan2.setContent("Zone");
    insertion_cyan2.setAlignment(rcll_draw::CenterCenter);
    insertion_cyan2.setBorderColor(rcll_draw::C_CYAN_DARK);
    insertion_cyan2.setBackgroundColor(rcll_draw::C_CYAN_DARK);
    insertion_cyan2.setFrontColor(rcll_draw::C_BLACK);
    insertion_cyan2.setFontSize(0.75);

    insertion_magenta1.setContent("Insertion");
    insertion_magenta1.setAlignment(rcll_draw::CenterCenter);
    insertion_magenta1.setBorderColor(rcll_draw::C_MAGENTA_DARK);
    insertion_magenta1.setBackgroundColor(rcll_draw::C_MAGENTA_DARK);
    insertion_magenta1.setFrontColor(rcll_draw::C_BLACK);
    insertion_magenta1.setFontSize(0.75);

    insertion_magenta2.setContent("Zone");
    insertion_magenta2.setAlignment(rcll_draw::CenterCenter);
    insertion_magenta2.setBorderColor(rcll_draw::C_MAGENTA_DARK);
    insertion_magenta2.setBackgroundColor(rcll_draw::C_MAGENTA_DARK);
    insertion_magenta2.setFrontColor(rcll_draw::C_BLACK);
    insertion_magenta2.setFontSize(0.75);
}

rcll_draw::GameField::~GameField(){

}

void rcll_draw::GameField::setPhase(rcll_draw::GamePhase gamephase){
    this->gamephase = gamephase;
    for (size_t i = 0; i < machine_markers.size(); i++){
        machine_markers[i].setPhase(gamephase);
    }
}


void rcll_draw::GameField::setGeometry(int x, int y, int w, int h){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;

    w0 = w;
    h0 = h;
    x0 = x + w / 2;
    y0 = y;
}

void rcll_draw::GameField::setLayout(double field_w, double field_h, int zones_x, int zones_y, std::vector<int> insertion_zones){
    // get the optimal field size
    int gapsize = h0 * 0.02;
    pixel_per_meter = (h0 - 2 * gapsize) / field_h;
    h0 = (int)field_h * pixel_per_meter;
    w0 = (int)field_w * pixel_per_meter;
    x0 = x0;
    y0 = y0 + gapsize;

    // calculate the vertical grid lines
    int zone_w = w0 / (zones_x);
    for (int i = 0; i < zones_x + 1; i++){
        Line line;
        int x_line = x0 - (zones_x / 2) * zone_w + i * zone_w;
        line.setLineByPoints(x_line, y0 + h0, x_line, y0);
        line.setBorderSize(1);
        line.setBorderColor(rcll_draw::C_BLACK);
        zone_lines.push_back(line);
    }

    // calculate the horizontal grid lines
    int zone_h = h0 / (zones_y);
    for (int i = 0; i < zones_y + 1; i++){
        Line line;
        int y_line = y0 + h0 - i * zone_h;
        line.setLineByPoints(x0 - w0 / 2, y_line, x0 + w0 / 2, y_line);
        line.setBorderSize(1);
        line.setBorderColor(rcll_draw::C_BLACK);
        zone_lines.push_back(line);
    }

    // calculate the zone labels
    for (int zx = -zones_x/2; zx <= zones_x/2; zx++){
        if (zx == 0){
            continue;
        }
        for (int zy = 1; zy <= zones_y; zy++){
            BoxLabel zone_label;
            int unsigned_zone_number = 10 * abs(zx) + zy;

            bool cancel = false;
            for (size_t i = 0; i < insertion_zones.size(); i++){
                if (insertion_zones[i] == unsigned_zone_number){
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
                zone_label.setPos(x0 + zx * zone_w, y0 + zy * zone_h - zone_h / 4);
            } else {
                zone_label.setContent(" M-Z" + std::to_string(unsigned_zone_number));
                zone_label.setAlignment(rcll_draw::CenterCenter);
                zone_label.setPos(x0 + (zx - 1) * zone_w, y0 + zy * zone_h - zone_h / 4);
            }
            zone_label.setBorderColor(rcll_draw::C_TRANSPARENT);
            zone_label.setBackgroundColor(rcll_draw::C_TRANSPARENT);
            zone_label.setFrontColor(rcll_draw::C_BLACK);
            zone_label.setBorderSize(1);
            zone_label.setFontSize(0.5);

            zone_label.setSize(zone_w, zone_h / 4);


            zone_names.push_back(zone_label);
        }
    }

    background.setPos(x0 - w0 / 2 - gapsize, y0 - gapsize);
    background.setSize(w0 + 2 * gapsize, h0 + 2 * gapsize);
    background2.setPos(x0 - w0 /2, y0);
    background2.setSize(w0, h0);

    int h1 = (zone_h - 2 * gapsize) / 2;
    insertion_cyan1.setSize(3 * zone_w - 2 * gapsize, h1);
    insertion_cyan2.setSize(3 * zone_w - 2 * gapsize, h1);
    insertion_magenta1.setSize(3 * zone_w - 2 * gapsize, h1);
    insertion_magenta2.setSize(3 * zone_w - 2 * gapsize, h1);

    insertion_cyan1.setPos(x0 - zones_x/2 * zone_w + gapsize, y0 + zone_h / 2 - h1);
    insertion_cyan2.setPos(x0 - zones_x/2 * zone_w + gapsize, y0 + zone_h / 2);
    insertion_magenta1.setPos(x0 + (zones_x/2 - 3) * zone_w + gapsize, y0 + zone_h / 2 - h1);
    insertion_magenta2.setPos(x0 + (zones_x/2 - 3) * zone_w + gapsize, y0 + zone_h / 2);
}

void rcll_draw::GameField::addWall(double x1, double y1, double x2, double y2){
    Line line;
    line.setLineByPoints(x0 + x1 * pixel_per_meter, y0 + y1 * pixel_per_meter, x0 + x2 * pixel_per_meter, y0 + y2 * pixel_per_meter);
    line.setBorderSize(4);
    line.setBorderColor(rcll_draw::C_BLACK);
    walls.push_back(line);
}

size_t rcll_draw::GameField::addRobot(std::string name, int id, rcll_draw::Team team){
    RobotMarker robot(team);
    robot.setOrigin(x0, y0, pixel_per_meter);
    robot.setRobotParams(name, id, 0.5);
    robot_markers.push_back(robot);
    return robot_markers.size() - 1;
}

void rcll_draw::GameField::setRobotPos(double x, double y, double yaw, size_t index){
    if (index >= 0 && index < robot_markers.size()){
        robot_markers[index].setPos(x, y, yaw);
    }
}

size_t rcll_draw::GameField::addMachine(std::string name, rcll_draw::Team team){
    MachineMarker machine(team);
    machine.setOrigin(x0, y0, pixel_per_meter);
    machine.setMachineParams(name, 0.7, 0.35);
    machine_markers.push_back(machine);
    return machine_markers.size() - 1;
}

void rcll_draw::GameField::setMachinePos(double x, double y, double yaw, size_t index){
    if (index >= 0 && index < machine_markers.size()){
        machine_markers[index].setPos(x, y, yaw);
    }
}

void rcll_draw::GameField::draw(cv::Mat &mat){
    //background.draw(mat);
    background2.draw(mat);

    for (size_t i = 0; i < walls.size(); i++){
        walls[i].draw(mat);
    }

    if (gamephase == rcll_draw::SETUP){
        for (size_t i = 0; i < zone_lines.size(); i++){
            zone_lines[i].draw(mat);
        }

        for (size_t i = 0; i < zone_names.size(); i++){
            zone_names[i].draw(mat);
        }
    }

    insertion_cyan1.draw(mat);
    insertion_cyan2.draw(mat);
    insertion_magenta1.draw(mat);
    insertion_magenta2.draw(mat);

    for (size_t i = 0; i < robot_markers.size(); i++){
        robot_markers[i].draw(mat);
    }

    for (size_t i = 0; i < machine_markers.size(); i++){
        machine_markers[i].draw(mat);
    }

    if (gamephase == rcll_draw::SETUP){
        cv::line(mat, cv::Point(x0, y0), cv::Point(x0 - 30, y0), rcll_draw::getColor(rcll_draw::C_RED), 2, 8, 0);
        cv::line(mat, cv::Point(x0, y0), cv::Point(x0, y0 + 30), rcll_draw::getColor(rcll_draw::C_GREEN_LIGHT), 2, 8, 0);
    }
}
