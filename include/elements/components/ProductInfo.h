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
#ifndef RCLL_PRODUCT_INFO_H
#define RCLL_PRODUCT_INFO_H

#include <util.h>

#include <rcll_vis_msgs/Product.h>

#include <ProductLabel.h>

namespace rcll_draw {
    class ProductInfo {
    public:
        ProductInfo();
        ProductInfo(rcll_draw::Team team, int displayed_products = 4);
        ~ProductInfo();

        void setGeometry(int x, int y, double scale);

        int getW(double scale = 1.0);
        int getH(double scale = 1.0);

        void setProducts(std::vector<rcll_vis_msgs::Product> &products);
        void prepare_draw();
        bool move();
        void draw(cv::Mat &mat, bool show_element_border = false);

        static void getKeys(std::map<std::string, rcll_draw::ProductLabel> &mapping, std::vector<std::string> &keys);

    private:
        int x, y = 0;
        int w = 1640, h = 540;
        int w_product = 400;
        int gapsize = 20;
        double scale = 1.0;
        cv::Mat origin;
        cv::Mat canvas;
        cv::Mat crop;

        size_t displayed_products = 4;
        Team team;

        std::vector<std::string> keys;
        std::vector<std::string> display;
        std::map<std::string, ProductLabel> pls_products;

        int shift = 0;
        int wait = 0;
    };
}

#endif
