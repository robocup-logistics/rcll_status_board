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

    for (size_t i = 0; i < product_labels.size(); i++){
        size_t product_index = page * product_labels.size() + i;
        if (product_index < products.size()){
            product_labels[i].setProduct(products[product_index]);
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
