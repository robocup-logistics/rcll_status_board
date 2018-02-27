#include <TeamHeaderPanel.h>

// TeamHeaderPanel ####################################################################
rcll_draw::TeamHeaderPanel::TeamHeaderPanel(){

}

rcll_draw::TeamHeaderPanel::~TeamHeaderPanel(){

}

void rcll_draw::TeamHeaderPanel::setTeam(std::string team_name, rcll_draw::Team team_color){
    if (team_color == rcll_draw::CYAN){
        blbl_header_color.setContent("CYAN");
        blbl_header_color.setFrontColor(rcll_draw::C_CYAN_DARK);
    } else if (team_color == rcll_draw::MAGENTA){
        blbl_header_color.setContent("MAGENTA");
        blbl_header_color.setFrontColor(rcll_draw::C_MAGENTA_DARK);
    } else {
        blbl_header_color.setContent("");
        blbl_header_color.setFrontColor(rcll_draw::C_BLACK);
    }
    blbl_header_name.setContent(team_name);

    blbl_header_color.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_header_name.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_header_color.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_header_name.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_header_color.setBorderColor(rcll_draw::C_WHITE);
    blbl_header_name.setBorderColor(rcll_draw::C_WHITE);
}

void rcll_draw::TeamHeaderPanel::setGeometry(int x, int y, int w, int h){
    blbl_header_color.setPos(x, y);
    blbl_header_name.setPos(x, y + 2 * h / 3);

    blbl_header_color.setSize(w, 2 * h / 3);
    blbl_header_name.setSize(w, h / 3);

    blbl_header_color.setFontSize(2.0);
    blbl_header_name.setFontSize(1.0);
}

void rcll_draw::TeamHeaderPanel::draw(cv::Mat &mat){
    blbl_header_color.draw(mat);
    blbl_header_name.draw(mat);
}
