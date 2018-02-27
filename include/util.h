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
        C_GREEN = 8,
        C_RED = 9,
        C_YELLOW = 10,
        C_BLUE = 11,
        C_TRANSPARENT = 12
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
    std::vector<std::string> splitString(std::string s);
    cv::Mat readImage(std::string file);
    void mergeImages(cv::Mat &dst, cv::Mat &src, cv::Scalar alpha_color, int x_dst, int y_dst);
    std::string getFile(int number, int type);

    void setImagePath(std::string path);
    std::string getImagePath();

    static std::string image_path;
}

#endif
