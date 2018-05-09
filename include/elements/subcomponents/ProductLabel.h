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

#ifndef RCLL_PRODUCT_LABEL_H
#define RCLL_PRODUCT_LABEL_H

#include <util.h>

#include <rcll_vis_msgs/Product.h>

#include <BoxLabel.h>
#include <Image.h>

namespace rcll_draw {

    struct ProductPlan {
        int complexity; //0-3
        int base;   // 1=RED, 2=BLACK, 3=SILVER
        int ring1;  // 1=BLUE, 2=GREEN, 3=ORANGE, 4=YELLOW
        int ring2;  // 1=BLUE, 2=GREEN, 3=ORANGE, 4=YELLOW
        int ring3;  // 1=BLUE, 2=GREEN, 3=ORANGE, 4=YELLOW
        int cap;    // 1=BLACK, 2=GREY

        int status_product; // 0=not started, 1=construction, 2=delivery, 3=completed
        int status_base;
        int status_ring1;
        int status_ring2;
        int status_ring3;
        int status_cap;
    };

    struct ProductInformation {
        int product_id;
        int quantity_id;
        ProductPlan plan;
        double progress;
        int deadline;
        int points;
        int points_max;
    };

    class ProductLabel {
    public:
        ProductLabel();
        ~ProductLabel();

        void setGeometry(int x, int y);
        int getW();
        int getH();
        void setProduct(rcll_vis_msgs::Product &product, rcll_draw::Team team);
        cv::Mat createProductImage(ProductPlan plan);
        void draw(cv::Mat &mat);

    private:
        int x, y = 0;
        int w = 400, h = 540;
        cv::Mat origin;

        BoxLabel blbl_name;
        BoxLabel blbl_progress;
        BoxLabel blbl_deadline;
        BoxLabel blbl_points;
        Image img_product;
        std::vector<Image> img_step_progress;
        Image img_product_progress;
    };
}

#endif
