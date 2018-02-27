#include <Label.h>

// Label functions #################################################
rcll_draw::Label::Label(){

}

rcll_draw::Label::~Label(){

}

void rcll_draw::Label::setContent(std::string content){
    this->content = content;
}

void rcll_draw::Label::setPos(int x, int y){
    this->x = x;
    this->y = y;
}

void rcll_draw::Label::setFrontColor(Color c){
    this->frontcolor = c;
}

void rcll_draw::Label::setFont(std::string font, double fontsize){
    this->font = font;
    this->fontsize = fontsize;
}

void rcll_draw::Label::setFontSize(double fontsize){
    this->fontsize = fontsize;
}

void rcll_draw::Label::setAlignment(Alignment alignment){
    this->alignment = alignment;
}

void rcll_draw::Label::draw(cv::Mat &mat){
    int baseline = 0;
    cv::Size textsize = cv::getTextSize(content, BASIC_FONT, fontsize, 2 * fontsize, &baseline);
    cv::Point org;
    if (alignment == Alignment::TopLeft){
        org = cv::Point(x, y + textsize.height);
    } else if (alignment == Alignment::TopCenter){
        org = cv::Point(x - textsize.width / 2, y + textsize.height);
    } else if (alignment == Alignment::TopRight){
        org = cv::Point(x - textsize.width, y + textsize.height);
    } else if (alignment == Alignment::CenterLeft){
        org = cv::Point(x, y + textsize.height / 2);
    } else if (alignment == Alignment::CenterCenter){
        org = cv::Point(x - textsize.width / 2, y + textsize.height / 2);
    } else if (alignment == Alignment::CenterRight){
        org = cv::Point(x - textsize.width, y + textsize.height / 2);
    } else if (alignment == Alignment::BottomLeft){
        org = cv::Point(x, y);
    } else if (alignment == Alignment::BottomCenter){
        org = cv::Point(x - textsize.width / 2, y);
    } else if (alignment == Alignment::BottomRight){
        org = cv::Point(x - textsize.width, y);
    } else {
        org = cv::Point(0, 0);
    }
    cv::putText(mat, content, org, BASIC_FONT, fontsize, getColor(frontcolor), 2 * fontsize, 8, false);
}
