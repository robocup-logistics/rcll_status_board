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
        void setProduct(ProductInformation pi, int index);
        void setProductsCount(size_t count);
        void paging();
        void draw(cv::Mat &mat);

    private:
        std::vector<ProductInformation> products;
        std::vector<ProductLabel> product_labels;

        ProductInformation empty_product;

        size_t page = 0;
    };
}

#endif
