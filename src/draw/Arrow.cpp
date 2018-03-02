#include <Arrow.h>

// Arrow functions #################################################
rcll_draw::Arrow::Arrow(){

}

rcll_draw::Arrow::~Arrow(){

}

void rcll_draw::Arrow::setArrowByLength(int x1, int y1, double angle, double length){
    cv::Point p;
    polygon.clear();

    /*
              p3
              *
             / \
            /   \
           *--*--*
         p4  p1  p2
    */

    // Rotational CenterPoint p1
    p.x = x1;
    p.y = y1;
    polygon.push_back(p);

    // Bottom Right Point p2
    p.x = x1 + length / 4 * cos(angle);
    p.y = y1 + length / 4 * sin(angle);
    polygon.push_back(p);

    // Top Center Point p3
    p.x = x1 + length * cos(angle + M_PI / 2);
    p.y = y1 + length * sin(angle + M_PI / 2);
    polygon.push_back(p);

    // Bottom Left Point p4
    p.x = x1 - length / 4 * cos(angle);
    p.y = y1 - length / 4 * sin(angle);
    polygon.push_back(p);
}

void rcll_draw::Arrow::setColor(Color c){
    this->color = c;
}

void rcll_draw::Arrow::draw(cv::Mat &mat){
    cv::fillConvexPoly(mat, this->polygon, rcll_draw::getColor(this->color), 8, 0);
}
