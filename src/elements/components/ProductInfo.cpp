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

#include <ProductInfo.h>

// ProductInfo ####################################################################
rcll_draw::ProductInfo::ProductInfo(){
    product_labels.resize(4);
    origin = cv::Mat(h, w, CV_8UC4);

    empty_product.product_id = 0;
}

rcll_draw::ProductInfo::~ProductInfo(){

}

void rcll_draw::ProductInfo::setGeometry(int x, int y, double scale){
    this->x = x;
    this->y = y;
    this->scale = scale;

    int w1 = (w - 3 * gapsize) / 4;
    for(size_t i = 0; i < product_labels.size(); i++){
        product_labels[i].setGeometry(i * (gapsize + w1), 0, w1, h);
        product_labels[i].setProduct(empty_product);
    }
}

int rcll_draw::ProductInfo::getW(double scale){
    return (int)((double)w * scale);
}

int rcll_draw::ProductInfo::getH(double scale){
    return (int)((double)h * scale);
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

    for (size_t i = 0; i < product_labels.size(); i++){
        size_t product_index = page * product_labels.size() + i;
        if (product_index < products.size()){
            product_labels[i].setProduct(products[product_index]);
        } else {
            product_labels[i].setProduct(empty_product);
        }
    }
}

void rcll_draw::ProductInfo::draw(cv::Mat &mat, bool show_element_border){
    cv::rectangle(origin, cv::Point(0, 0), cv::Point (w-1, h-1), rcll_draw::getColor(rcll_draw::C_WHITE), CV_FILLED);

    for (size_t i = 0; i < product_labels.size(); i++){
        product_labels[i].draw(origin);
    }

    if (show_element_border){
        cv::rectangle(origin, cv::Point(0, 0), cv::Point (w-1, h-1), rcll_draw::getColor(rcll_draw::C_RED), 1);
    }

    rcll_draw::mergeImages(mat, origin, x, y, scale);
}
