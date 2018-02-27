#ifndef RCLL_IMAGE_H
#define RCLL_IMAGE_H

#include <util.h>

namespace rcll_draw {
    class Image {
    public:
        Image();
        ~Image();

        void setPos(int x, int y); // origin is top left corner
        void setScale(double s);
        void setImage(cv::Mat image);
        int getW();
        int getH();
        void loadImage(std::string file);
        void setAngle(double angle);
        void draw(cv::Mat &mat);
        void draw(cv::Mat &mat, cv::Scalar alpha_color);

    protected:
        std::string path = "";
        cv::Mat image;
        int x, y = 0;
        double s;
        double angle = 0;
    };
}

#endif
