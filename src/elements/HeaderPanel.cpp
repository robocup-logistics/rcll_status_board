#include <HeaderPanel.h>

// HeaderPanel ####################################################################
rcll_draw::HeaderPanel::HeaderPanel(std::string content, rcll_draw::Team team){
    blbl_header.setContent(content);
    blbl_header.setAlignment(rcll_draw::Alignment::CenterCenter);
    blbl_header.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_header.setBorderColor(rcll_draw::C_WHITE);
    if (team == rcll_draw::CYAN){
        blbl_header.setFrontColor(rcll_draw::C_CYAN_DARK);
    } else if (team == rcll_draw::MAGENTA){
        blbl_header.setFrontColor(rcll_draw::C_MAGENTA_DARK);
    } else {
        blbl_header.setFrontColor(rcll_draw::C_BLACK);
    }
}

rcll_draw::HeaderPanel::~HeaderPanel(){

}

void rcll_draw::HeaderPanel::setGeometry(int y, int w, int h){
    blbl_header.setPos(0, y);
    blbl_header.setSize(w, h);
    blbl_header.setFontSize(2.0);
}

void rcll_draw::HeaderPanel::draw(cv::Mat &mat){
    blbl_header.draw(mat);
}
