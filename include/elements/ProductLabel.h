#ifndef RCLL_PRODUCT_LABEL_H
#define RCLL_PRODUCT_LABEL_H

#include <util.h>
#include <BoxLabel.h>
#include <Image.h>

namespace rcll_draw {

    struct Product {
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

    class ProductLabel {
    public:
        ProductLabel();
        ~ProductLabel();

        void setGeometry(int x, int y, int w, int h);
        void setProduct(int id, Product plan, double blbl_progress, int blbl_deadline, int blbl_points, int points_max);
        cv::Mat createProductImage(Product plan);
        void draw(cv::Mat &mat);

    private:
        BoxLabel blbl_name;
        BoxLabel blbl_progress;
        BoxLabel blbl_deadline;
        BoxLabel blbl_points;
        Image product;
        std::vector<Image> img_step_progress;
        Image img_product_progress;

        int id;
    };
}

#endif
