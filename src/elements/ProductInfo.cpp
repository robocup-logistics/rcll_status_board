#include <ProductInfo.h>

// ProductInfo ####################################################################
rcll_draw::ProductInfo::ProductInfo(){
    product_labels.resize(4);

    empty_product.product_id = 0;
}

rcll_draw::ProductInfo::~ProductInfo(){

}

void rcll_draw::ProductInfo::setGeometry(int x, int y, int w, int h, int gapsize){
    int w1 = (w - 3 * gapsize) / 4;
    product_labels[0].setGeometry(x, y, w1, h);
    product_labels[1].setGeometry(x + gapsize + w1, y, w1, h);
    product_labels[2].setGeometry(x + 2 * (gapsize + w1), y, w1, h);
    product_labels[3].setGeometry(x + 3 * (gapsize + w1), y, w1, h);

    product_labels[0].setProduct(empty_product);
    product_labels[1].setProduct(empty_product);
    product_labels[2].setProduct(empty_product);
    product_labels[3].setProduct(empty_product);
}

void rcll_draw::ProductInfo::setProduct(ProductInformation pi, int index){
    if (index >= 0 && index < (int)products.size()){
        products[index] = pi;
    }
}

void rcll_draw::ProductInfo::setProductsCount(size_t count){
    products.clear();
    products.resize(count);
}

void rcll_draw::ProductInfo::paging(){
    size_t pages = products.size() / product_labels.size();

    if (products.size() % product_labels.size() > 0){
        pages +=1;
    }

    page++;
    if (page >= pages){
        page = 0;
    }

    ROS_INFO(" paging to page %zu of %zu; products: %zu", page + 1, pages, products.size());

    for (size_t i = 0; i < product_labels.size(); i++){
        size_t product_index = page * product_labels.size() + i;
        if (product_index < products.size()){
            product_labels[i].setProduct(products[product_index]);
            ROS_INFO(" showing product %i.%i", products[product_index].product_id, products[product_index].quantity_id);
        } else {
            product_labels[i].setProduct(empty_product);
        }
    }
}

void rcll_draw::ProductInfo::draw(cv::Mat &mat){
    for (size_t i = 0; i < product_labels.size(); i++){
        product_labels[i].draw(mat);
    }
}
