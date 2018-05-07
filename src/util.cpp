/*
The MIT License (MIT)

Copyright (c) 2017-2018 Florian Eith <florian.eith@web.de>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

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
        case C_GREEN_LIGHT: return cv::Scalar(0, 210, 0);
        case C_RED: return cv::Scalar(0, 0, 210);
        case C_YELLOW: return cv::Scalar(0, 210, 255);
        case C_BLUE: return cv::Scalar(255, 0, 0);
        case C_GREEN_DARK: return cv::Scalar(0, 128, 0);
        default: return cv::Scalar(255, 255, 255);
    }
}

bool rcll_draw::getBoolSignal(ros::Time time, ros::Rate rate){
    return (fmod(time.toSec(), 2 * rate.expectedCycleTime().toSec()) <= rate.expectedCycleTime().toSec());
}

bool rcll_draw::getBoolSignal(ros::Time time, int second){
        return (((int)time.toSec() % second) == 0);
    }

bool rcll_draw::getSignalOnceEveryXSeconds(ros::Time time, int intervall, ros::Rate basic_rate){
    return ((int)(time.toSec() - basic_rate.expectedCycleTime().toSec()) != (int)(time.toSec())) && rcll_draw::getBoolSignal(time, intervall);
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

void rcll_draw::mergeImages(cv::Mat &dst, cv::Mat &src, int x_dst, int y_dst){
    for (int y = 0; y < src.rows; ++y){
        if (y_dst + y <= dst.rows){
            for (int x = 0; x < src.cols; ++x){
                if (x_dst + x <= dst.cols){
                    cv::Vec4b & dst_pixel = dst.at<cv::Vec4b>(y_dst + y, x_dst + x);
                    cv::Vec4b & src_pixel = src.at<cv::Vec4b>(y, x);
                    dst_pixel[0] = src_pixel[0];
                    dst_pixel[1] = src_pixel[1];
                    dst_pixel[2] = src_pixel[2];
                    dst_pixel[3] = src_pixel[3];
                }
            }
        }
    }
}

void rcll_draw::mergeImages(cv::Mat &dst, cv::Mat &src, int x_dst, int y_dst, double scale){
    cv::Mat tmp;
    cv::resize(src, tmp, cv::Size(), scale, scale, cv::INTER_CUBIC);
    rcll_draw::mergeImages(dst, tmp, x_dst, y_dst);
    tmp.resize(0);
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
    } else if (type == 4){ // status image for products
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
    } else if (type == 5){ // status image for machine exploration reports
        if (number == 0){ // 0=questionmark
            return "questionmark.ppm";
        } else if (number == 1){ // 1=checkmark
            return "checkmark.ppm";
        } else if (number == 2){ // 2=red cross
            return "red_cross.ppm";
        }
    }

    return "";
}

std::string rcll_draw::getGameStateStr(int gamestate){
    switch (gamestate){
        case 0: return "INIT";
        case 1: return "WAIT START";
        case 2: return "RUNNING";
        case 3: return "PAUSED";
        default: return "UNKNOWN";
    }
}

std::string rcll_draw::getGamePhaseStr(int gamephase){
    switch (gamephase){
        case 0: return "PRE GAME";
        case 10: return "SETUP";
        case 20: return "EXPLORATION";
        case 30: return "PRODUCTION";
        case 40: return "POST GAME";
        default: return "UNKNOWN";
    }
}

void rcll_draw::setImagePath(std::string path){
    rcll_draw::image_path = path;
}

std::string rcll_draw::getImagePath(){
    return rcll_draw::image_path;
}
