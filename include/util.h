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

#ifndef RCLL_UTIL_H
#define RCLL_UTIL_H
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define BASIC_FONT cv::FONT_HERSHEY_DUPLEX //cv::FONT_HERSHEY_DUPLEX

namespace rcll_draw {
    enum Team {
        CYAN = 0,
        MAGENTA = 1,
        NO_TEAM = 2
    };

    enum GamePhase {
        PRE_GAME = 0,
        SETUP = 10,
        EXPLORATION = 20,
        PRODUCTION = 30,
        POST_GAME = 40
    };

    enum Color {
        C_BLACK = 0,
        C_GREY_DARK = 1,
        C_GREY_LIGHT = 2,
        C_WHITE = 3,
        C_CYAN_DARK = 4,
        C_CYAN_LIGHT = 5,
        C_MAGENTA_DARK = 6,
        C_MAGENTA_LIGHT = 7,
        C_GREEN_LIGHT = 8,
        C_RED = 9,
        C_YELLOW = 10,
        C_BLUE = 11,
        C_GREEN_DARK = 12,
        C_TRANSPARENT = 13
    };

    enum Alignment {
        TopLeft = 0,
        TopRight = 1,
        TopCenter = 2,
        CenterLeft = 3,
        CenterRight = 4,
        CenterCenter = 5,
        BottomLeft = 6,
        BottomRight = 7,
        BottomCenter = 8
    };

    enum LineType {
        Continuous = 0,
        Dotted = 1,
        Dashed = 2,
        Arrowed = 3
    };

    cv::Scalar getColor(Color color);
    bool getBoolSignal(ros::Time time, ros::Rate rate);
    bool getBoolSignal(ros::Time time, int second);
    bool getSignalOnceEveryXSeconds(ros::Time time, int intervall, ros::Rate basic_rate);
    std::vector<std::string> splitString(std::string s);
    cv::Mat readImage(std::string file);
    void mergeImages(cv::Mat &dst, cv::Mat &src, cv::Scalar alpha_color, int x_dst, int y_dst);
    void mergeImages(cv::Mat &dst, cv::Mat &src, int x_dst, int y_dst);
    std::string getFile(int number, int type);
    std::string getGamePhaseStr(int gamephase);
    std::string getGameStateStr(int gamestate);

    void setImagePath(std::string path);
    std::string getImagePath();

    static std::string image_path;
    static const double machine_length = 0.7;
    static const double machine_width = 0.35;
}

#endif
