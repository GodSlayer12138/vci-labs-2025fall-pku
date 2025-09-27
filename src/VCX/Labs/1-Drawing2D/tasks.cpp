#include <random>

#include <spdlog/spdlog.h>

#include "Labs/1-Drawing2D/tasks.h"

using VCX::Labs::Common::ImageRGB;

namespace VCX::Labs::Drawing2D {
    /******************* 1.Image Dithering *****************/
    void DitheringThreshold(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringRandomUniform(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        static std::default_random_engine            e;
        static std::uniform_real_distribution<float> u(0.0, 1.0);
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color     = input.At(x, y);
                float     threshold = u(e);
                output.At(x, y) = {
                    color.r > threshold ? 1 : 0,
                    color.g > threshold ? 1 : 0,
                    color.b > threshold ? 1 : 0,
                };
            }
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {
        // your code here:
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                color           = color + noise.At(x, y) - glm::vec3(0.5);
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        static float table[3][3] {
            6. / 9., 8. / 9., 4. / 9., 1. / 9., 0. / 9., 3. / 9., 5. / 9., 2. / 9., 7. / 9.
        };

        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j) {
                        output.At(3 * x + i, 3 * y + j) = {
                            color.r > table[i][j] ? 1 : 0,
                            color.g > table[i][j] ? 1 : 0,
                            color.b > table[i][j] ? 1 : 0,
                        };
                    }
            }
    }

    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        output = input;
        for (std::size_t y = 0; y < input.GetSizeY(); ++y)
            for (std::size_t x = 0; x < input.GetSizeX(); ++x) {
                glm::vec3 color    = output.At(x, y);
                float     inscale  = (color.r + color.g + color.b) / 3;
                float     outscale = inscale > 0.5 ? 1 : 0;
                output.At(x, y) = { outscale, outscale, outscale };
                inscale = inscale - outscale;
                if (x + 1 < input.GetSizeX()) {
                    color = output.At(x + 1, y);
                    output.At(x + 1, y) = { color.r + inscale * 7 / 16, color.g + inscale * 7 / 16, color.b + inscale * 7 / 16 };
                }
                if (y + 1 < output.GetSizeY()) {
                    if (x > 0) {
                        color = output.At(x - 1, y + 1);
                        output.At(x - 1, y + 1) = { color.r + inscale * 3 / 16, color.g + inscale * 3 / 16, color.b + inscale * 3 / 16 };
                    }
                    color = output.At(x, y + 1);
                    output.At(x, y + 1) = { color.r + inscale * 5 / 16, color.g + inscale * 5 / 16, color.b + inscale * 5 / 16 };
                    if (x + 1 < output.GetSizeX()) {
                        color = output.At(x + 1, y + 1);
                        output.At(x + 1, y + 1) = { color.r + inscale * 1 / 16, color.g + inscale * 1 / 16, color.b + inscale * 1 / 16 };
                    }
                }
            }
    }

    /******************* 2.Image Filtering *****************/
    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        static auto const getOrZeroAt {
            [&input](std::size_t const x, std::size_t const y) -> glm::vec4 {
                if (x < 0 || x >= input.GetSizeX()) return {};
                if (y < 0 || y >= input.GetSizeY()) return {};
                return { input.At(x, y), 1 };
            }
        };
        // clang-format off
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                auto const acc {
                    getOrZeroAt(x - 1, y - 1) +
                    getOrZeroAt(x    , y - 1) +
                    getOrZeroAt(x + 1, y - 1) +
                    getOrZeroAt(x - 1, y    ) +
                    getOrZeroAt(x    , y    ) +
                    getOrZeroAt(x + 1, y    ) +
                    getOrZeroAt(x - 1, y + 1) +
                    getOrZeroAt(x    , y + 1) +
                    getOrZeroAt(x + 1, y + 1)
                };
                output.At(x, y) = {acc.r / acc.w, acc.g / acc.w, acc.b / acc.w};
            }
        // clang-format on
    }

    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        static auto const getOrNaNAt {
            [&input](std::size_t const x, std::size_t const y) -> glm::vec4 {
                if (x < 0 || x >= input.GetSizeX()) return { 0, 0, 0, NAN };
                if (y < 0 || y >= input.GetSizeY()) return { 0, 0, 0, NAN };
                return { input.At(x, y), 1 };
            }
        };
        // clang-format off
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                auto const gradx {
                    getOrNaNAt(x - 1, y - 1) * -1.f +
                    getOrNaNAt(x - 1, y    ) * -2.f +
                    getOrNaNAt(x - 1, y + 1) * -1.f +
                    getOrNaNAt(x + 1, y - 1) *  1.f +
                    getOrNaNAt(x + 1, y    ) *  2.f +
                    getOrNaNAt(x + 1, y + 1) *  1.f
                };
                auto const grady {
                    getOrNaNAt(x - 1, y - 1) * -1.f +
                    getOrNaNAt(x    , y - 1) * -2.f +
                    getOrNaNAt(x + 1, y - 1) * -1.f +
                    getOrNaNAt(x - 1, y + 1) *  1.f +
                    getOrNaNAt(x    , y + 1) *  2.f +
                    getOrNaNAt(x + 1, y + 1) *  1.f
                };
                if (! std::isnan(gradx.w))
                    output.At(x, y) = {
                        std::sqrt(gradx.r * gradx.r + grady.r * grady.r),
                        std::sqrt(gradx.g * gradx.g + grady.g * grady.g),
                        std::sqrt(gradx.b * gradx.b + grady.b * grady.b),
                    };
            }
        // clang-format on
    }

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) {
        output               = inputBack;
        std::size_t width    = inputFront.GetSizeX();
        std::size_t height   = inputFront.GetSizeY();

        std::vector<glm::vec3> g(width * height);
        // set boundary condition
        for (std::size_t y = 0; y < height; ++y) {
            // set boundary for (0, y), your code: g[y * width] = ?
            g[y * width] = inputBack.At(unsigned(offset.x), offset.y + y) - inputFront.At(0, y);
            // set boundary for (width - 1, y), your code: g[y * width + width - 1] = ?
            g[y * width + width - 1] = inputBack.At(offset.x + width - 1, offset.y + y) - inputFront.At(width - 1, y);
        }
        for (std::size_t x = 0; x < width; ++x) {
            // set boundary for (x, 0), your code: g[x] = ?
            g[x] = inputBack.At(offset.x + x, unsigned(offset.y)) - inputFront.At(x, 0);
            // set boundary for (x, height - 1), your code: g[(height - 1) * width + x] = ?
            g[(height - 1) * width + x] = inputBack.At(offset.x + x, offset.y + height - 1) - inputFront.At(x, height - 1);
        }

        // Use Gauss-Seidel Iterative Method to solve Ag = 0
        for (int iter = 0; iter < 8000; ++iter) {
            for (std::size_t y = 1; y < height - 1; ++y)
                for (std::size_t x = 1; x < width - 1; ++x) {
                    g[y * width + x] = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                    g[y * width + x] = g[y * width + x] * glm::vec3(0.25);
                }
        }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                glm::vec3 color = g[y * width + x] + inputFront.At(x, y);
                output.At(x + offset.x, y + offset.y) = color;
            }
    }

    extern void DrawLineImpl(ImageRGB &, glm::vec3, int, int, int, int);
    /******************* 4. Line Drawing *****************/
    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
        // your code here:
        DrawLineImpl(canvas, color, p0.x, p0.y, p1.x, p1.y);
    }

    /******************* 5. Triangle Drawing *****************/
    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        // your code here:
        int xmin { std::min(std::min(p0.x, p1.x), p2.x) };
        int xmax { std::max(std::max(p0.x, p1.x), p2.x) };
        int ymin { std::min(std::min(p0.y, p1.y), p2.y) };
        int ymax { std::max(std::max(p0.y, p1.y), p2.y) };
        for (std::size_t x = xmin; x <= xmax; ++x)
            for (std::size_t y = ymin; y <= ymax; ++y) {
                static auto const PointOnWhichSide {
                    [](
                        glm::ivec2 const p0,
                        glm::ivec2 const p1,
                        glm::ivec2 const p2) -> float {
                        // clang-format off
                        return
                            (p0.x - p2.x) * (p1.y - p2.y) -
                            (p1.x - p2.x) * (p0.y - p2.y);
                        // clang-format on
                    }
                };

                static auto const PointInTriangle {
                    [](
                        glm::ivec2 const p0,
                        glm::ivec2 const p1,
                        glm::ivec2 const p2,
                        glm::ivec2 const point) -> bool {
                        auto const side0  = PointOnWhichSide(point, p0, p1);
                        auto const side1  = PointOnWhichSide(point, p1, p2);
                        auto const side2  = PointOnWhichSide(point, p2, p0);
                        auto const hasNeg = (side0 < 0) || (side1 < 0) || (side2 < 0);
                        auto const hasPos = (side0 > 0) || (side1 > 0) || (side2 > 0);
                        return ! (hasNeg && hasPos);
                    }
                };

                if (PointInTriangle(p0, p1, p2, glm::ivec2(x, y)))
                    canvas.At(x, y) = color;
            }
    }

    /******************* 6. Image Supersampling *****************/
    void Supersample(
        ImageRGB &       output,
        ImageRGB const & input,
        int              rate) {
        // your code here:
        float const scaleX = float(input.GetSizeX()) / output.GetSizeX();
        float const scaleY = float(input.GetSizeY()) / output.GetSizeY();
        for (std::size_t x = 0; x < output.GetSizeX(); ++x)
            for (std::size_t y = 0; y < output.GetSizeY(); ++y) {
                glm::vec3 color { 0, 0, 0 };
                for (int dx = 0; dx < rate; ++dx) {
                    for (int dy = 0; dy < rate; ++dy) {
                        float fx = (x + (dx + 0.5) / rate) * scaleX;
                        float fy = (y + (dy + 0.5) / rate) * scaleY;
                        auto sx1 = std::max(std::size_t(fx - 0.5), std::size_t(0));
                        auto sx2 = std::min(sx1 + 1, input.GetSizeX() - 1);
                        auto sy1 = std::max(std::size_t(fy - 0.5), std::size_t(0));
                        auto sy2 = std::min(sy1 + 1, input.GetSizeY() - 1);
                        float wx = std::clamp(fx - sx1 - 0.5f, 0.0f, 1.0f);
                        float wy = std::clamp(fy - sy1 - 0.5f, 0.0f, 1.0f);
                        color   = color + input.At(sx1, sy1) * (1 - wx) * (1 - wy) + input.At(sx2, sy1) * wx * (1 - wy) + input.At(sx1, sy2) * (1 - wx) * wy + input.At(sx2, sy2) * wx * wy;
                    }
                }
                output.At(x, y) = color * glm::vec3(1.0f / (rate * rate));
            }
    }

    /******************* 7. Bezier Curve *****************/
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
        // your code here:
        if (points.size() == 1)
            return points[0];
        std::vector<glm::vec2> next;
        next.reserve(points.size() - 1);
        for (int i = 0; i < points.size() - 1; ++i)
            next.push_back(points[i] * (1 - t) + points[i + 1] * t);
        return CalculateBezierPoint(next, t);
    }

    /******************* 8. Image Warping *****************/
    void WarpImage(
        ImageRGB &                         dest,
        std::array<glm::ivec2, 16> const & destPoints,
        ImageRGB const &                   src,
        std::array<glm::ivec2, 16> const & srcPoints) {
        // your code here:
        static auto const BarycentricInterPolation {
            [](
                glm::ivec2 const p0,
                glm::ivec2 const p1,
                glm::ivec2 const p2,
                glm::ivec2 const point) -> glm::vec3 {
                float a1 = float((point.y - p0.y) * (p2.x - p0.x) - (point.x - p0.x) * (p2.y - p0.y)) / ((p1.y - p0.y) * (p2.x - p0.x) - (p1.x - p0.x) * (p2.y - p0.y));
                float a2 = float((point.y - p0.y) * (p1.x - p0.x) - (point.x - p0.x) * (p1.y - p0.y)) / ((p2.y - p0.y) * (p1.x - p0.x) - (p2.x - p0.x) * (p1.y - p0.y));
                return { 1 - a1 - a2, a1, a2 };
            }
        };
        
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                glm::ivec2 destLeftTop     = destPoints[i * 4 + j];
                glm::ivec2 destRightTop    = destPoints[i * 4 + j + 1];
                glm::ivec2 destRightBottom = destPoints[i * 4 + j + 5];
                glm::ivec2 destLeftBottom  = destPoints[i * 4 + j + 4];
                glm::ivec2 srcLeftTop      = srcPoints[i * 4 + j];
                glm::ivec2 srcRightTop     = srcPoints[i * 4 + j + 1];
                glm::ivec2 srcRightBottom  = srcPoints[i * 4 + j + 5];
                glm::ivec2 srcLeftBottom   = srcPoints[i * 4 + j + 4];
                glm::ivec2 minBound {
                    std::min(destLeftTop.x, destLeftBottom.x),
                    std::min(destLeftTop.y, destRightTop.y)
                };
                glm::ivec2 maxBound {
                    std::max(destRightTop.x, destRightBottom.x),
                    std::max(destLeftBottom.y, destRightBottom.y)
                };
                ImageRGB mask(maxBound.x - minBound.x + 1, maxBound.y - minBound.y + 1);
                DrawTriangleFilled(mask, { 1, 1, 1 }, destLeftTop - minBound, destRightTop - minBound, destLeftBottom - minBound);
                for (std::size_t x = 0; x < mask.GetSizeX(); ++x)
                    for (std::size_t y = 0; y < mask.GetSizeY(); ++y)
                        if (glm::vec3(mask.At(x, y)).r > 0) {
                            glm::ivec2 destPos = minBound + glm::ivec2 { int(x), int(y) };
                            glm::vec3 coeffs = BarycentricInterPolation(destLeftTop, destRightTop, destLeftBottom, destPos);
                            int srcX = int(std::round(coeffs.r * srcLeftTop.x + coeffs.g * srcRightTop.x + coeffs.b * srcLeftBottom.x));
                            int srcY = int(std::round(coeffs.r * srcLeftTop.y + coeffs.g * srcRightTop.y + coeffs.b * srcLeftBottom.y));
                            dest.At(destPos.x, destPos.y) = src.At(srcX, srcY);
                        }
                mask.Fill({ 0, 0, 0 });
                DrawTriangleFilled(mask, { 1, 1, 1 }, destRightTop - minBound, destRightBottom - minBound, destLeftBottom - minBound);
                for (std::size_t x = 0; x < mask.GetSizeX(); ++x)
                    for (std::size_t y = 0; y < mask.GetSizeY(); ++y)
                        if (glm::vec3(mask.At(x, y)).r > 0) {
                            glm::ivec2 destPos = minBound + glm::ivec2 { int(x), int(y) };
                            glm::vec3 coeffs = BarycentricInterPolation(destRightTop, destRightBottom, destLeftBottom, destPos);
                            int srcX = int(std::round(coeffs.r * srcRightTop.x + coeffs.g * srcRightBottom.x + coeffs.b * srcLeftBottom.x));
                            int srcY = int(std::round(coeffs.r * srcRightTop.y + coeffs.g * srcRightBottom.y + coeffs.b * srcLeftBottom.y));
                            dest.At(destPos.x, destPos.y) = src.At(srcX, srcY);
                        }
            }
        }
    }
} // namespace VCX::Labs::Drawing2D