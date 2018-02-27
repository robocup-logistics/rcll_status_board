#include <Line.h>

// Line functions #################################################
rcll_draw::Line::Line(){

}

rcll_draw::Line::~Line(){

}

void rcll_draw::Line::setLine(int x1, int y1, int x2, int y2){
    this->x1 = x1;
    this->y1 = y1;
    this->x2 = x2;
    this->y2 = y2;
}

void rcll_draw::Line::setLine(int x1, int y1, double angle, double length){
    this->x1 = x1;
    this->y1 = y1;
    this->x2 = x1 + length * cos(angle);
    this->y2 = y1 + length * sin(angle);
}

void rcll_draw::Line::setBorderColor(Color c){
    this->bordercolor = c;
}

void rcll_draw::Line::setBorderSize(int bordersize){
    this->bordersize = bordersize;
}

void rcll_draw::Line::setLineType(LineType linetype){
    this->linetype = linetype;
}

void rcll_draw::Line::draw(cv::Mat &mat){
    if (linetype == LineType::Continuous){
        cv::line(mat, cv::Point(x1, y1), cv::Point(x2, y2), getColor(bordercolor), bordersize, 8, 0);
    } else if (linetype == LineType::Dotted){
        // TODO
    } else if (linetype == LineType::Dashed){
        // TODO
    } else if (linetype == LineType::Arrowed){
        // TODO
    }

}


// HLine functions #################################################
rcll_draw::HLine::HLine(){

}

rcll_draw::HLine::~HLine(){

}

void rcll_draw::HLine::setLine(int x1, int y, int x2){
    Line::setLine(x1, y, x2, y);
}


// VLine functions #################################################
rcll_draw::VLine::VLine(){

}

rcll_draw::VLine::~VLine(){

}

void rcll_draw::VLine::setLine(int x, int y1, int y2){
    Line::setLine(x, y1, x, y2);
}
