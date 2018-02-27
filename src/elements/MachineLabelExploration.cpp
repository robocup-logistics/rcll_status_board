#include <MachineLabelExploration.h>

// MachineLabelExploration ####################################################################

rcll_draw::MachineLabelExploration::MachineLabelExploration(){
    blbl_machinename.setAlignment(rcll_draw::CenterLeft);
    blbl_status1.setAlignment(rcll_draw::CenterCenter);
    blbl_status2.setAlignment(rcll_draw::CenterCenter);

    blbl_machinename.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_status1.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    blbl_status2.setBackgroundColor(rcll_draw::C_GREY_LIGHT);

    blbl_machinename.setBorderColor(rcll_draw::C_WHITE);
    blbl_status1.setBorderColor(rcll_draw::C_WHITE);
    blbl_status2.setBorderColor(rcll_draw::C_WHITE);

    blbl_machinename.setFontSize(0.9);
    blbl_status1.setFontSize(0.7);
    blbl_status2.setFontSize(0.7);

    blbl_machinename.setFrontColor(rcll_draw::C_WHITE);
    blbl_status1.setFrontColor(rcll_draw::C_BLACK);
    blbl_status2.setFrontColor(rcll_draw::C_BLACK);

    img_status1.setImage(rcll_draw::readImage(rcll_draw::getFile(4,4)));
    img_status2.setImage(rcll_draw::readImage(rcll_draw::getFile(4,4)));
}

rcll_draw::MachineLabelExploration::~MachineLabelExploration(){

}

void rcll_draw::MachineLabelExploration::setGeometry(int x, int y, int w, int h){
    blbl_machinename.setSize(w * 0.4, h);
    blbl_status1.setSize(w * 0.3, h);
    blbl_status2.setSize(w * 0.3, h);
    img_status1.setScale(0.25);
    img_status2.setScale(0.25);

    blbl_machinename.setPos(x, y);
    blbl_status1.setPos(x + w * 0.4, y);
    blbl_status2.setPos(x + w * 0.7, y);
    img_status1.setPos(x + w * 0.625, y + (h - img_status1.getH())/2);
    img_status2.setPos(x + w * 0.925, y + (h - img_status2.getH())/2);
}

void rcll_draw::MachineLabelExploration::setMachineName(std::string name, int index){
    blbl_machinename.setContent(name);
    if (index % 2 == 0){
        blbl_status1.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
        blbl_status2.setBackgroundColor(rcll_draw::C_GREY_LIGHT);
    } else {
        blbl_status1.setBackgroundColor(rcll_draw::C_GREY_DARK);
        blbl_status2.setBackgroundColor(rcll_draw::C_GREY_DARK);
    }
}

void rcll_draw::MachineLabelExploration::setMachineStatus(int status1, int status2){
    if (status1 == 0){
        blbl_status1.setContent("unreported");
        img_status1.setImage(rcll_draw::readImage(rcll_draw::getFile(4,4)));
    } else if (status1 == 1){
        blbl_status1.setContent("correct");
        img_status1.setImage(rcll_draw::readImage(rcll_draw::getFile(5,4)));
    } else if (status1 == 2){
        blbl_status1.setContent("wrong");
        img_status1.setImage(rcll_draw::readImage(rcll_draw::getFile(6,4)));
    } else {
        blbl_status1.setContent("unknown");
        img_status1.setImage(rcll_draw::readImage(""));
    }

    if (status2 == 0){
        blbl_status2.setContent("unreported");
        img_status2.setImage(rcll_draw::readImage(rcll_draw::getFile(4,4)));
    } else if (status2 == 1){
        blbl_status2.setContent("correct");
        img_status2.setImage(rcll_draw::readImage(rcll_draw::getFile(5,4)));
    } else if (status2 == 2){
        blbl_status2.setContent("wrong");
        img_status2.setImage(rcll_draw::readImage(rcll_draw::getFile(6,4)));
    } else {
        blbl_status2.setContent("unknown");
        img_status2.setImage(rcll_draw::readImage(""));
    }
}

void rcll_draw::MachineLabelExploration::setHeader(std::string status1, std::string status2){
    blbl_machinename.setContent("");
    blbl_status1.setContent(status1);
    blbl_status2.setContent(status2);

    blbl_machinename.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_status1.setBackgroundColor(rcll_draw::C_BLACK);
    blbl_status2.setBackgroundColor(rcll_draw::C_BLACK);

    blbl_machinename.setFrontColor(rcll_draw::C_WHITE);
    blbl_status1.setFrontColor(rcll_draw::C_WHITE);
    blbl_status2.setFrontColor(rcll_draw::C_WHITE);

    blbl_machinename.setFontSize(1.0);
    blbl_status1.setFontSize(1.0);
    blbl_status2.setFontSize(1.0);

    img_status1.setImage(rcll_draw::readImage(""));
    img_status2.setImage(rcll_draw::readImage(""));
}

void rcll_draw::MachineLabelExploration::draw(cv::Mat &mat){
    blbl_machinename.draw(mat);
    blbl_status1.draw(mat);
    blbl_status2.draw(mat);
    img_status1.draw(mat, rcll_draw::getColor(rcll_draw::C_WHITE));
    img_status2.draw(mat, rcll_draw::getColor(rcll_draw::C_WHITE));
}
