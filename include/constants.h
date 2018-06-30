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

#ifndef RCLL_CONSTANTS_H
#define RCLL_CONSTANTS_H

namespace rcll_draw {
    enum Team {
        CYAN = 0,
        MAGENTA = 1,
        NO_TEAM = 2
    };

    enum GameState {
        INIT = 0,
        WAIT_START = 1,
        RUNNING = 2,
        PAUSED = 3
    };

    enum GamePhase {
        PRE_GAME = 0,
        SETUP = 10,
        EXPLORATION = 20,
        PRODUCTION = 30,
        POST_GAME = 40
    };

    enum Color {
        C_BLACK = 0,
        C_GREY_DARK = 1,
        C_GREY_LIGHT = 2,
        C_WHITE = 3,
        C_CYAN_DARK = 4,
        C_CYAN_LIGHT = 5,
        C_MAGENTA_DARK = 6,
        C_MAGENTA_LIGHT = 7,
        C_GREEN_LIGHT = 8,
        C_RED = 9,
        C_YELLOW = 10,
        C_BLUE = 11,
        C_GREEN_DARK = 12,
        C_TRANSPARENT = 13
    };

    enum Alignment {
        TopLeft = 0,
        TopRight = 1,
        TopCenter = 2,
        CenterLeft = 3,
        CenterRight = 4,
        CenterCenter = 5,
        BottomLeft = 6,
        BottomRight = 7,
        BottomCenter = 8
    };

    enum LineType {
        Continuous = 0,
        Dotted = 1,
        Dashed = 2,
        Arrowed = 3
    };
}

#endif
