#include <drawing.h>

#define BASIC_FONT cv::FONT_HERSHEY_DUPLEX //cv::FONT_HERSHEY_DUPLEX

// static functions #################################################
cv::Scalar rcll_draw::getColor(Color color){
    switch (color){
        case C_BLACK: return cv::Scalar(0, 0, 0);
        case C_GREY_DARK: return cv::Scalar(120, 120, 120);
        case C_GREY_LIGHT: return cv::Scalar(171, 171, 171);
        case C_WHITE: return cv::Scalar(255, 255, 255);
        case C_CYAN_DARK: return cv::Scalar(128, 128, 0);
        case C_CYAN_LIGHT: return cv::Scalar(255, 255, 0);
        case C_MAGENTA_DARK: return cv::Scalar(183, 0, 183);
        case C_MAGENTA_LIGHT: return cv::Scalar(255, 0, 255);
        case C_GREEN: return cv::Scalar(0, 210, 0);
        case C_RED: return cv::Scalar(0, 0, 210);
        case C_YELLOW: return cv::Scalar(0, 210, 255);
        default: return cv::Scalar(255, 255, 255);
    }
}

bool rcll_draw::getBoolSignal(ros::Time time, ros::Rate rate){
    return (fmod(time.toSec(), 2 * rate.expectedCycleTime().toSec()) <= rate.expectedCycleTime().toSec());
}

std::vector<std::string> rcll_draw::splitString(std::string s){
    std::vector<std::string> list;

    if (s == ""){ return list; }

    std::string laststring;
    for (size_t i = 0; i < s.size(); i++){
        std::string teststring = s.substr(i, 1);
        if (teststring == " "){
            list.push_back(laststring);
            laststring = "";
        } else {
            laststring += s[i];
        }
    }
    list.push_back(laststring);
    return list;
}

cv::Mat rcll_draw::readImage(std::string file){
    cv::Mat result;
    if (file != ""){
        cv::cvtColor(cv::imread(rcll_draw::Image::getImagePath() + file), result, CV_BGR2BGRA);
    } else {
        result = cv::Mat(10, 10, CV_8UC4);
        cv::rectangle(result, cv::Point(0,0), cv::Point(result.cols, result.rows), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED);
    }
    return result;
}

void rcll_draw::mergeImages(cv::Mat &dst, cv::Mat &src, cv::Scalar alpha_color, int x_dst, int y_dst){
    for (int y = 0; y < src.rows; ++y){
        if (y_dst + y <= dst.rows){
            for (int x = 0; x < src.cols; ++x){
                if (x_dst + x <= dst.cols){
                    cv::Vec4b & dst_pixel = dst.at<cv::Vec4b>(y_dst + y, x_dst + x);
                    cv::Vec4b & src_pixel = src.at<cv::Vec4b>(y, x);
                    // if pixel is alphacolor
                    if (!(src_pixel[0] == alpha_color[0] && src_pixel[1] == alpha_color[1] && src_pixel[2] == alpha_color[2])){
                        // source pixel is not alpha color
                        // draw source over target
                        dst_pixel[0] = src_pixel[0];
                        dst_pixel[1] = src_pixel[1];
                        dst_pixel[2] = src_pixel[2];
                        dst_pixel[3] = src_pixel[3];
                    }
                }
            }
        }
    }
}

std::string rcll_draw::getFile(int number, int type){
    if (type == 1){ // base
        if (number == 1){ // red base
            return "base_red.ppm";
        } else if (number == 2){ // black base
            return "base_black.ppm";
        } else if (number == 3){ // silver base
            return "base_silver.ppm";
        }
    } else if (type == 2){ // ring
        if (number == 1){ // blue ring
            return "ring_blue.ppm";
        } else if (number == 2){ // green ring
            return "ring_green.ppm";
        } else if (number == 3){ // orange ring
            return "ring_orange.ppm";
        } else if (number == 4){ // yellow ring
            return "ring_yellow.ppm";
        }
    } else if (type == 3){ // cap
        if (number == 1){ // black cap
            return "cap_black.ppm";
        } else if (number == 2){ // grey cap
            return "cap_grey.ppm";
        }
    } else if (type == 4){ // status image
        if (number == 0){ // 0=not started
            return "";
        } else if (number == 1){ // 1=construction
            return "gears.ppm";
        } else if (number == 2){ // 2=delivery
            return "arrow.ppm";
        } else if (number == 3){ // 3=completed
            return "checkmark.ppm";
        } else if (number == 4){ // 4=questionmark
            return "questionmark.ppm";
        } else if (number == 5){ // 3=checkmark
            return "checkmark.ppm";
        } else if (number == 6){ // 3=red cross
            return "red_cross.ppm";
        }
    }

    return "";
}


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


// BoxLabel functions #################################################
rcll_draw::BoxLabel::BoxLabel(){

}

rcll_draw::BoxLabel::~BoxLabel(){

}

void rcll_draw::BoxLabel::setSize(int w, int h){
    this->w = w;
    this->h = h;
}

void rcll_draw::BoxLabel::setBackgroundColor(Color c){
    this->backgroundcolor = c;
}

void rcll_draw::BoxLabel::setBorderColor(Color c){
    this->bordercolor = c;
}

void rcll_draw::BoxLabel::setBorderSize(int bordersize){
    this->bordersize = bordersize;
}

void rcll_draw::BoxLabel::draw(cv::Mat &mat){
    if (backgroundcolor != rcll_draw::C_TRANSPARENT){
        // draw background rectangle
        cv::rectangle(mat, cv::Point(x, y), cv::Point(x + w, y + h), getColor(backgroundcolor), CV_FILLED, 8, 0);
    }

    if (bordercolor != rcll_draw::C_TRANSPARENT){
        // draw border rectangle
        cv::rectangle(mat, cv::Point(x, y), cv::Point(x + w, y + h), getColor(bordercolor), bordersize, 8, 0);
    }

    if (content != ""){
        int baseline = 0;
        cv::Size textsize = cv::getTextSize(content, BASIC_FONT, fontsize, 2 * fontsize, &baseline);
        cv::Point org;
        if (alignment == Alignment::TopLeft){
            org = cv::Point(x, y + textsize.height + h*0.15);
        } else if (alignment == Alignment::TopCenter){
            org = cv::Point(x + (w - textsize.width) / 2, y + textsize.height + h*0.15);
        } else if (alignment == Alignment::TopRight){
            org = cv::Point(x + w - textsize.width, y + textsize.height + h*0.15);
        } else if (alignment == Alignment::CenterLeft){
            org = cv::Point(x, y + (h + textsize.height) / 2);
        } else if (alignment == Alignment::CenterCenter){
            org = cv::Point(x + (w - textsize.width) / 2, y + (h + textsize.height) / 2);
        } else if (alignment == Alignment::CenterRight){
            org = cv::Point(x + w - textsize.width, y + (h + textsize.height) / 2);
        } else if (alignment == Alignment::BottomLeft){
            org = cv::Point(x, y + h - h*0.15);
        } else if (alignment == Alignment::BottomCenter){
            org = cv::Point(x + (w - textsize.width) / 2, y + h - h*0.15);
        } else if (alignment == Alignment::BottomRight){
            org = cv::Point(x + w - textsize.width, y + h - h*0.15);
        } else {
            org = cv::Point(0, 0);
        }
        cv::putText(mat, content, org, BASIC_FONT, fontsize, getColor(frontcolor), 2 * fontsize, 8, false);
    }
}

// MultilineBoxLabel functions #################################################
rcll_draw::MultilineBoxLabel::MultilineBoxLabel(){

}

rcll_draw::MultilineBoxLabel::~MultilineBoxLabel(){

}

void rcll_draw::MultilineBoxLabel::draw(cv::Mat &mat){
    if (backgroundcolor != rcll_draw::C_TRANSPARENT){
        // draw background rectangle
        cv::rectangle(mat, cv::Point(x, y), cv::Point(x + w, y + h), getColor(backgroundcolor), CV_FILLED, 8, 0);
    }

    if (bordercolor != rcll_draw::C_TRANSPARENT){
        // draw border rectangle
        cv::rectangle(mat, cv::Point(x, y), cv::Point(x + w, y + h), getColor(bordercolor), bordersize, 8, 0);
    }

    // split the text into lines
    std::vector<std::string> words = rcll_draw::splitString(content);
    std::vector<std::string> lines;
    lines.push_back("");
    for (size_t i = 0; i < words.size(); i++){
        int baseline = 0;
        std::string line = lines[lines.size() - 1] + " " + words[i];
        cv::Size textsize = cv::getTextSize(line, BASIC_FONT, fontsize, 2 * fontsize, &baseline);
        if (textsize.width < w){
            lines[lines.size() - 1] = line;
        } else {
            lines.push_back(" " + words[i]);
        }
    }

    // print the lines
    for (size_t i = 0; i < lines.size(); i++){
        if (lines[i] != ""){
            int baseline = 0;
            cv::Size textsize = cv::getTextSize(lines[i], BASIC_FONT, fontsize, 2 * fontsize, &baseline);
            cv::Point org = cv::Point(x, y + textsize.height + h*0.15 + i * (h*0.85)/lines.size());
            cv::putText(mat, lines[i], org, BASIC_FONT, fontsize, getColor(frontcolor), 2 * fontsize, 8, false);
        }
    }
}


// Circle functions #################################################
rcll_draw::Circle::Circle(){

}

rcll_draw::Circle::~Circle(){

}

void rcll_draw::Circle::setPos(int x, int y){
    this->x = x;
    this->y = y;
}

void rcll_draw::Circle::setSize(double radius){
    this->r = radius;
}

void rcll_draw::Circle::setBackgroundColor(Color c){
    this->backgroundcolor = c;
}

void rcll_draw::Circle::setBorderColor(Color c){
    this->bordercolor = c;
}

void rcll_draw::Circle::setBorderSize(int bordersize){
    this->bordersize = bordersize;
}

void rcll_draw::Circle::draw(cv::Mat &mat){
    if (backgroundcolor != rcll_draw::C_TRANSPARENT){
        // draw background circle
        cv::circle(mat, cv::Point(x, y), r, getColor(backgroundcolor), CV_FILLED, 8, 0);
    }
    if (bordercolor != rcll_draw::C_TRANSPARENT){
        // draw border circle
        cv::circle(mat, cv::Point(x, y), r, getColor(bordercolor), bordersize, 8, 0);
    }
}


// Image functions #################################################
std::string rcll_draw::Image::image_path = "";

rcll_draw::Image::Image(){
    this->image = cv::Mat(0, 0, CV_8UC4);
}

rcll_draw::Image::~Image(){

}

void rcll_draw::Image::setPos(int x, int y){
    this->x = x;
    this->y = y;
}

void rcll_draw::Image::setScale(double s){
    this->s = s;
}

int rcll_draw::Image::getW(){
    return (double)this->image.cols * s;
}

int rcll_draw::Image::getH(){
    return (double)this->image.rows * s;
}

void rcll_draw::Image::setImage(cv::Mat image){
    this->path = "";
    this->image = image;
}

void rcll_draw::Image::loadImage(std::string file){
    this->path = rcll_draw::Image::getImagePath() + file;

    this->image = cv::imread(path, CV_LOAD_IMAGE_UNCHANGED);
    if(!this->image.data){
        ROS_WARN("Could not open or find the picture at '%s'", path.c_str());
        return;
    }
}

void rcll_draw::Image::setAngle(double angle){
    this->angle = angle;
}

void rcll_draw::Image::draw(cv::Mat &mat){
    cv::Mat tmp2;
    try {
        cv::resize(this->image, tmp2, cv::Size(), s, s, cv::INTER_CUBIC);
        if (tmp2.data){
            cv::Rect rect(x, y, tmp2.cols, tmp2.rows);
            cv::Mat roi = mat(rect);
            tmp2.copyTo(roi);
        } else {
            ROS_WARN("Could not open or find the picture at '%s'", path.c_str());
        }
    } catch (cv::Exception &e){
        ROS_ERROR("%s", e.what());
        ROS_INFO("x=%i y=%i w=%i h=%i", x, y, tmp2.cols, tmp2.rows);
    }
}

void rcll_draw::Image::draw(cv::Mat &mat, cv::Scalar alpha_color){
    cv::Mat tmp2;
    cv::resize(this->image, tmp2, cv::Size(), s, s, cv::INTER_LINEAR);
    rcll_draw::mergeImages(mat, tmp2, alpha_color, x, y);
}

void rcll_draw::Image::setImagePath(std::string path){
    rcll_draw::Image::image_path = path;
}

std::string rcll_draw::Image::getImagePath(){
    return rcll_draw::Image::image_path;
}


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
