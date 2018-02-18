#ifndef RCLL_DRAWING_H
#define RCLL_DRAWING_H
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace rcll_draw {
    enum Team {
        CYAN = 0,
        MAGENTA = 1,
        NO_TEAM = 2
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
        C_TRANSPARENT = 11
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

    class Label {
    public:
        Label();
        ~Label();

        void setContent(std::string content);
        void setPos(int x, int y); // origin is top left corner
        void setFrontColor(Color c);
        void setFont(std::string font, double fontsize);
        void setFontSize(double fontsize);
        void setAlignment(Alignment alignment);
        void draw(cv::Mat &mat);

    protected:
        int x, y = 0;
        std::string content = "";
        Color frontcolor = C_BLACK;
        std::string font;
        double fontsize = 12.0;
        Alignment alignment = CenterCenter;
    };

    class BoxLabel : public Label {
    public:
        BoxLabel();
        ~BoxLabel();

        void setSize(int w, int h);
        void setBackgroundColor(Color c);
        void setBorderColor(Color c);
        void setBorderSize(int bordersize);
        void draw(cv::Mat &mat);

    protected:
        int w, h = 10;
        Color bordercolor = C_BLACK;
        Color backgroundcolor = C_WHITE;
        int bordersize = 1;

    };

    class MultilineBoxLabel : public BoxLabel {
    public:
        MultilineBoxLabel();
        ~MultilineBoxLabel();

        void draw(cv::Mat &mat);
    };

    class Circle {
    public:
        Circle();
        ~Circle();

        void setPos(int x, int y); // origin is center or circle
        void setSize(double radius);
        void setBackgroundColor(Color c);
        void setBorderColor(Color c);
        void setBorderSize(int bordersize);
        void draw(cv::Mat &mat);

    protected:
        int x, y = 0;
        double r = 1;
        Color bordercolor = C_BLACK;
        Color backgroundcolor = C_WHITE;
        int bordersize = 1;
    };

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

        static void setImagePath(std::string path);
        static std::string getImagePath();

    protected:
        std::string path = "";
        cv::Mat image;
        int x, y = 0;
        double s;
        double angle = 0;

        static std::string image_path;

    };

    class Line {
    public:
        Line();
        ~Line();
        void setLine(int x1, int y1, int x2, int y2);
        void setLine(int x1, int y1, double angle, double length);
        void setBorderColor(Color c);
        void setBorderSize(int bordersize);
        void setLineType(LineType linetype);
        void draw(cv::Mat &mat);

    protected:
        int x1, y1, x2, y2 = 0;
        Color bordercolor = C_BLACK;
        int bordersize = 1;
        LineType linetype = Continuous;
    };

    class HLine : public Line {
    public:
        HLine();
        ~HLine();

        void setLine(int x1, int y, int x2);
    };

    class VLine : public Line {
    public:
        VLine();
        ~VLine();

        void setLine(int x, int y1, int y2);
    };
}

#endif
