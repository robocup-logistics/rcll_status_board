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

#include <MultilineBoxLabel.h>

// MultilineBoxLabel functions #################################################
rcll_draw::MultilineBoxLabel::MultilineBoxLabel(){

}

rcll_draw::MultilineBoxLabel::~MultilineBoxLabel(){

}

void rcll_draw::MultilineBoxLabel::setLines(int lines){
    this->lines = lines;
}

void rcll_draw::MultilineBoxLabel::draw(cv::Mat &mat){
    if (backgroundcolor != rcll_draw::C_TRANSPARENT){
        // draw background rectangle
        cv::rectangle(mat, cv::Point(x, y), cv::Point(x + w, y + h), getColor(backgroundcolor), CV_FILLED, 8, 0);
    }

    if (bordercolor != rcll_draw::C_TRANSPARENT){
        // draw border rectangle
        cv::rectangle(mat, cv::Point(x, y), cv::Point(x + w, y + h), getColor(bordercolor), bordersize, 8, 0);
    }

    // split the text into lines
    std::vector<std::string> words = rcll_draw::splitString(content);
    std::vector<std::string> lines_str;
    lines_str.push_back("");
    for (size_t i = 0; i < words.size(); i++){
        int baseline = 0;
        std::string line = lines_str[lines_str.size() - 1] + " " + words[i];
        cv::Size textsize = cv::getTextSize(line, BASIC_FONT, fontsize, 2 * fontsize, &baseline);
        if (textsize.width < w){
            lines_str[lines_str.size() - 1] = line;
        } else {
            lines_str.push_back(" " + words[i]);
        }
    }

    // print the lines
    for (size_t i = 0; i < lines_str.size(); i++){
        if (lines_str[i] != ""){
            int baseline = 0;
            cv::Size textsize = cv::getTextSize(lines_str[i], BASIC_FONT, fontsize, 2 * fontsize, &baseline);
            cv::Point org = cv::Point(x, y + textsize.height + h*0.15 + i * (h*0.85)/lines);
            cv::putText(mat, lines_str[i], org, BASIC_FONT, fontsize, getColor(frontcolor), 2 * fontsize, 8, false);
        }
    }
}
