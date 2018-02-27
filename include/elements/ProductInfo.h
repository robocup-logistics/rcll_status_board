#ifndef RCLL_PRODUCT_INFO_H
#define RCLL_PRODUCT_INFO_H

#include <util.h>
#include <ProductLabel.h>

namespace rcll_draw {
    class ProductInfo {
    public:
        ProductInfo();
        ~ProductInfo();

        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setProduct(int id, Product plan, double progress, int deadline, int points, int points_max, int index);
        void draw(cv::Mat &mat);

    private:
        std::vector<ProductLabel> products;
    };
}

#endif
