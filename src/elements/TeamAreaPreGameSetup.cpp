#include <TeamAreaPreGameSetup.h>
// TeamAreaPreGameSetup ####################################################################

rcll_draw::TeamAreaPreGameSetup::TeamAreaPreGameSetup(){
    blbl_versus.setContent("versus");
    blbl_text.setContent("Last preparations are arranged by the teams. The game will start shortly.");

    blbl_versus.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_text.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_versus.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_text.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_versus.setBorderColor(rcll_draw::C_WHITE);
    blbl_text.setBorderColor(rcll_draw::C_WHITE);

    blbl_versus.setFontSize(1.0);
    blbl_text.setFontSize(1.0);
}

rcll_draw::TeamAreaPreGameSetup::~TeamAreaPreGameSetup(){

}

void rcll_draw::TeamAreaPreGameSetup::setGameInfo(std::string gamestate, std::string gamephase, int time, int score_cyan, int score_magenta){
    game_info.setContent(gamestate, gamephase, time, score_cyan, score_magenta);
}

void rcll_draw::TeamAreaPreGameSetup::setTeams(std::string team_name_cyan, std::string team_name_magenta){
    thpan_team_cyan.setTeam(team_name_cyan, rcll_draw::CYAN);
    thpan_team_magenta.setTeam(team_name_magenta, rcll_draw::MAGENTA);
}

void rcll_draw::TeamAreaPreGameSetup::setGeometry(int x, int y, int w, int h, int gapsize){
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;

    thpan_team_cyan.setGeometry(x + w * 0.2, y + h * 0.3, w * 0.2, h * 0.1);
    thpan_team_magenta.setGeometry(x + w * 0.6, y + h * 0.3, w * 0.2, h * 0.1);
    blbl_versus.setSize(w * 0.1, h * 0.1 / 3);
    blbl_versus.setPos(x + w * 0.45, y + h * 0.3 + h * 0.1 * 2 / 3);
    blbl_text.setSize(w * 0.8, h * 0.1);
    blbl_text.setPos(x + w * 0.1, y + h * 0.6);
    game_info.setGeometry(x + w * 0.2, y, w * 0.6, h * 0.1);
}

void rcll_draw::TeamAreaPreGameSetup::draw(cv::Mat &mat){
    thpan_team_cyan.draw(mat);
    thpan_team_magenta.draw(mat);
    blbl_versus.draw(mat);
    blbl_text.draw(mat);
    game_info.draw(mat);
}
