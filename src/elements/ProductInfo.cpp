#include <ProductInfo.h>

// ProductInfo ####################################################################
rcll_draw::ProductInfo::ProductInfo(){
    products.resize(4);
}

rcll_draw::ProductInfo::~ProductInfo(){

}

void rcll_draw::ProductInfo::setGeometry(int x, int y, int w, int h, int gapsize){
    int w1 = (w - 3 * gapsize) / 4;
    products[0].setGeometry(x, y, w1, h);
    products[1].setGeometry(x + gapsize + w1, y, w1, h);
    products[2].setGeometry(x + 2 * (gapsize + w1), y, w1, h);
    products[3].setGeometry(x + 3 * (gapsize + w1), y, w1, h);
}

void rcll_draw::ProductInfo::setProduct(int id, Product plan, double progress, int deadline, int points, int points_max, int index){
    if (index >= 0 && index < 4){
        products[index].setProduct(id, plan, progress, deadline, points, points_max);
    }
}

void rcll_draw::ProductInfo::draw(cv::Mat &mat){
    for (size_t i = 0; i < products.size(); i++){
        products[i].draw(mat);
    }
}
