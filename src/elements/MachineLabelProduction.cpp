#include <MachineLabelProduction.h>

// MachineLabelProduction ####################################################################

rcll_draw::MachineLabelProduction::MachineLabelProduction(){
    blbl_machinename.setAlignment(rcll_draw::CenterLeft);
    blbl_state.setAlignment(rcll_draw::CenterCenter);
    blbl_lamp1.setAlignment(rcll_draw::CenterCenter);
    blbl_lamp2.setAlignment(rcll_draw::CenterCenter);

    blbl_machinename.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_state.setBackgroundColor(rcll_draw::C_TRANSPARENT);
    blbl_lamp1.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_lamp2.setBackgroundColor(rcll_draw::C_WHITE);
    blbl_border.setBackgroundColor(rcll_draw::C_TRANSPARENT);

    blbl_machinename.setBorderColor(rcll_draw::C_TRANSPARENT);
    blbl_state.setBorderColor(rcll_draw::C_TRANSPARENT);
    blbl_lamp1.setBorderColor(rcll_draw::C_WHITE);
    blbl_lamp2.setBorderColor(rcll_draw::C_WHITE);
    blbl_border.setBorderColor(rcll_draw::C_WHITE);

    blbl_machinename.setFontSize(0.9);
    blbl_state.setFontSize(0.7);
    blbl_lamp1.setFontSize(0.7);
    blbl_lamp2.setFontSize(0.7);

    blbl_machinename.setFrontColor(rcll_draw::C_WHITE);
    blbl_state.setFrontColor(rcll_draw::C_BLACK);
    blbl_lamp1.setFrontColor(rcll_draw::C_BLACK);
    blbl_lamp2.setFrontColor(rcll_draw::C_BLACK);
}

rcll_draw::MachineLabelProduction::~MachineLabelProduction(){

}

void rcll_draw::MachineLabelProduction::setGeometry(int x, int y, int w, int h){
    blbl_machinename.setPos(x, y);
    blbl_state.setPos(x + w * 0.7, y);
    blbl_lamp1.setPos(x + w * 0.7, y);
    blbl_lamp2.setPos(x + w * 0.7, y + h/2);
    blbl_border.setPos(x, y);

    blbl_machinename.setSize(w * 0.7, h);
    blbl_state.setSize(w * 0.3, h);
    blbl_lamp1.setSize(w * 0.3, h/2);
    blbl_lamp2.setSize(w * 0.3, h/2);
    blbl_border.setSize(w, h);
}

void rcll_draw::MachineLabelProduction::setMachineName(std::string name){
    blbl_machinename.setContent(name);
}

void rcll_draw::MachineLabelProduction::setMachineStatus(std::string status, Color lamp_top, Color lamp_bottom, std::string lamp_top_str, std::string lamp_bottom_str){
    blbl_state.setContent(status);
    blbl_lamp1.setContent(lamp_top_str);
    blbl_lamp2.setContent(lamp_bottom_str);
    lamp1_color = lamp_top;
    lamp2_color = lamp_bottom;
    blbl_lamp1.setBackgroundColor(lamp1_color);
    blbl_lamp2.setBackgroundColor(lamp2_color);
}

void rcll_draw::MachineLabelProduction::setFlashing(bool flashing){
    this->flashing = flashing;
}

void rcll_draw::MachineLabelProduction::draw(cv::Mat &mat){
    if (flashing){
        if (getBoolSignal(ros::Time::now(), ros::Rate(1.33))){
            blbl_lamp1.setBackgroundColor(lamp1_color);
            blbl_lamp2.setBackgroundColor(lamp2_color);
            blbl_lamp1.setBorderColor(lamp1_color);
            blbl_lamp2.setBorderColor(lamp2_color);
        } else {
            blbl_lamp1.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
            blbl_lamp2.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
            blbl_lamp1.setBorderColor(rcll_draw::C_GREY_LIGHT);
            blbl_lamp2.setBorderColor(rcll_draw::C_GREY_LIGHT);
        }
    } else {
        blbl_lamp1.setBackgroundColor(lamp1_color);
        blbl_lamp2.setBackgroundColor(lamp2_color);
        blbl_lamp1.setBorderColor(lamp1_color);
        blbl_lamp2.setBorderColor(lamp2_color);
    }

    blbl_machinename.draw(mat);
    blbl_lamp1.draw(mat);
    blbl_lamp2.draw(mat);
    blbl_state.draw(mat);
    blbl_border.draw(mat);
}
