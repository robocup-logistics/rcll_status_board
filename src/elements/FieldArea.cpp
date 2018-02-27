#include <FieldArea.h>
// FieldArea ####################################################################

rcll_draw::FieldArea::FieldArea(){
    game_info = HStatusPanel();
    team_cyan = TeamHeaderPanel();
    team_magenta = TeamHeaderPanel();
    gamefield = GameField();
}

rcll_draw::FieldArea::~FieldArea(){

}

void rcll_draw::FieldArea::setGeometry(int x, int y, int w, int h, int gapsize){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
    game_info.setGeometry(x + w * 0.2, y, w * 0.6, h * 0.1);
    team_cyan.setGeometry(x, y, w * 0.2, h * 0.1);
    team_magenta.setGeometry(x + w * 0.8, y, w * 0.2, h * 0.1);
    gamefield.setGeometry(x, y + h * 0.1 + gapsize, w, h * 0.9 - gapsize);
}

void rcll_draw::FieldArea::setGameInfo(std::string gamestate, std::string gamephase, int time, int score_cyan, int score_magenta){
    game_info.setContent(gamestate, gamephase, time, score_cyan, score_magenta);

    if (gamephase == "PRE GAME"){
        gamefield.setPhase(rcll_draw::PRE_GAME);
    } else if (gamephase == "SETUP"){
        gamefield.setPhase(rcll_draw::SETUP);
    } else if (gamephase == "EXPLORATION"){
        gamefield.setPhase(rcll_draw::EXPLORATION);
    } else if (gamephase == "PRODUCTION"){
        gamefield.setPhase(rcll_draw::PRODUCTION);
    } else if (gamephase == "POST GAME"){
        gamefield.setPhase(rcll_draw::POST_GAME);
    } else {
        gamefield.setPhase(rcll_draw::PRE_GAME);
    }
}

void rcll_draw::FieldArea::setLayout(double field_w, double field_h, int zones_x, int zones_y, std::vector<int> insertion_zones){
    gamefield.setLayout(field_w, field_h, zones_x, zones_y, insertion_zones);
}

void rcll_draw::FieldArea::setWalls(std::vector<double> wall_coordinates){
    for (size_t i = 0; i < wall_coordinates.size(); i+=4){
        if (i + 3 < wall_coordinates.size()){
            gamefield.addWall(wall_coordinates[i], wall_coordinates[i + 1], wall_coordinates[i + 2], wall_coordinates[i + 3]);
        }
    }
}

void rcll_draw::FieldArea::setTeam(std::string team_name, rcll_draw::Team team_color){
    if (team_color == rcll_draw::CYAN){
        team_cyan.setTeam(team_name, team_color);
    } else if (team_color == rcll_draw::MAGENTA){
        team_magenta.setTeam(team_name, team_color);
    } else {
        ROS_WARN("Invalid team color");
    }
}

void rcll_draw::FieldArea::draw(cv::Mat &mat){
    game_info.draw(mat);
    team_cyan.draw(mat);
    team_magenta.draw(mat);
    gamefield.draw(mat);
}
