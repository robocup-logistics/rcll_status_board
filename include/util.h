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

#include <constants.h>

#define BASIC_FONT cv::FONT_HERSHEY_DUPLEX //cv::FONT_HERSHEY_DUPLEX

namespace rcll_draw {
    cv::Scalar getColor(Color color);
    bool getBoolSignal(ros::Time time, ros::Rate rate);
    bool getBoolSignal(ros::Time time, int second);
    bool getSignalOnceEveryXSeconds(ros::Time time, int intervall, ros::Rate basic_rate);
    std::vector<std::string> splitString(std::string s);
    cv::Mat readImage(std::string file);
    void mergeImages(cv::Mat &dst, cv::Mat &src, cv::Scalar alpha_color, int x_dst, int y_dst, double scale);
    void mergeImages(cv::Mat &dst, cv::Mat &src, cv::Scalar alpha_color, int x_dst, int y_dst);
    void mergeImages(cv::Mat &dst, cv::Mat &src, int x_dst, int y_dst, double scale);
    void mergeImages(cv::Mat &dst, cv::Mat &src, int x_dst, int y_dst);

    std::string getFile(int number, int type);
    std::string getGamePhaseStr(int gamephase);
    std::string getGameStateStr(int gamestate);

    void setImagePath(std::string path);
    std::string getImagePath();

    static std::string image_path;
    static const double machine_length = 0.7;
    static const double machine_width = 0.35;
    static cv::Mat tmp;
}

#endif
