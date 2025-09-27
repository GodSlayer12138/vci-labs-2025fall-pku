#include <functional>

#include "Labs/1-Drawing2D/tasks.h"

using VCX::Labs::Common::ImageRGB;
using DrawPointCallback_t = std::function<void(int x, int y)>;
// note:
//    the following functions draw line from (0, 0) to (dy, dx)
//    and is decoupled with concrete renderer.

static void drawLineBresenhamCore45(
    const DrawPointCallback_t && drawPoint,
    const int                    dx,
    const int                    dy) {
    assert(dx >= 0 && dy >= 0 && dx >= dy);
    int x  = 0;
    int y  = 0;
    int xx = 0;
    int yy = 0;
    drawPoint(x, y);
    while (x <= dx) {
        drawPoint(x, y);
        const int new_yy = yy + dx;
        const int new_xx = xx + dy;
        ++x, xx = new_xx;
        if (new_yy <= new_xx)
            ++y, yy = new_yy;
    }
};

static void drawLineBresenhamCore90(
    DrawPointCallback_t && drawPoint,
    const int              dx,
    const int              dy) {
    assert(dx >= 0 && dy >= 0);
    if (dx >= dy)
        drawLineBresenhamCore45(std::move(drawPoint), dx, dy);
    else
        drawLineBresenhamCore45(
            [drawPoint = std::move(drawPoint)](const int row, const int col) {
                drawPoint(col, row);
            },
            dy,
            dx);
};

static void drawLineBresenhamCore180(
    DrawPointCallback_t && drawPoint,
    const int              dx,
    const int              dy) {
    assert(dy >= 0);
    if (dx >= 0)
        drawLineBresenhamCore90(std::move(drawPoint), dx, dy);
    else
        drawLineBresenhamCore90(
            [drawPoint = std::move(drawPoint)](const int row, const int col) {
                drawPoint(-row, col);
            },
            -dx,
            dy);
};

static void drawLineBresenhamCore360(
    DrawPointCallback_t && drawPoint,
    const int              dx,
    const int              dy) {
    if (dy >= 0)
        drawLineBresenhamCore180(std::move(drawPoint), dx, dy);
    else
        drawLineBresenhamCore180(
            [drawPoint = std::move(drawPoint)](const int row, const int col) {
                drawPoint(row, -col);
            },
            dx,
            -dy);
};

namespace VCX::Labs::Drawing2D {
    void DrawLineImpl(
        ImageRGB & canvas,
        glm::fvec3  color,
        int const   x0,
        int const   y0,
        int const   x1,
        int const   y1) {
        drawLineBresenhamCore360(
            [&](const int row, const int col) {
                canvas.At(x0 + row, y0 + col) = color;
            },
            x1 - x0,
            y1 - y0);
    }
}
