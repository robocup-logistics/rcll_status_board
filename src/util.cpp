#include <util.h>

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
        case C_BLUE: return cv::Scalar(255, 0, 0);
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
        cv::cvtColor(cv::imread(rcll_draw::getImagePath() + file), result, CV_BGR2BGRA);
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


void rcll_draw::setImagePath(std::string path){
    rcll_draw::image_path = path;
}

std::string rcll_draw::getImagePath(){
    return rcll_draw::image_path;
}
