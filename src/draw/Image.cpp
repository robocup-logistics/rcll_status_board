#include <Image.h>

// Image functions #################################################
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
    this->path = rcll_draw::getImagePath() + file;

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
