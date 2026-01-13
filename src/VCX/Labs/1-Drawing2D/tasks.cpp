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
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                double disturbance = float(rand()) / RAND_MAX - 0.5;
                output.At(x, y) = {
                    color.r + disturbance > 0.5 ? 1 : 0,
                    color.g + disturbance > 0.5 ? 1 : 0,
                    color.b + disturbance > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                double disturbance = noise.At(x, y)[0] - 0.5;
                output.At(x, y) = {
                    color.r + disturbance > 0.5 ? 1 : 0,
                    color.g + disturbance > 0.5 ? 1 : 0,
                    color.b + disturbance > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        const int ditherMatrix[3][3] = {
            { 6, 8, 4 },
            { 1, 0, 3 },
            { 5, 2, 7 }
        };
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
        {
            for (std::size_t y = 0; y < input.GetSizeY(); ++y)
            {
                double base = input.At(x, y)[0] * 9;
                for (int a = 0; a < 3; ++a)
                {
                    for (int b = 0; b < 3; ++b)
                    {
                        if (ditherMatrix[a][b] < base)
                            output.At(3 * x + a, 3 * y + b) = glm::vec3(1, 1, 1);
                        else
                            output.At(3 * x + a, 3 * y + b) = glm::vec3(0, 0, 0);
                    }
                }
            }
        }
    }

    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) {
        double errorMatrix[300][300] = {};
        for (std::size_t y = 0; y < input.GetSizeY(); ++y)
        {
            for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            {
                glm::vec3 color = input.At(x, y);
                color += errorMatrix[x][y];
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
                double error = color[0] - glm::vec3(output.At(x, y))[0];
                errorMatrix[x + 1][y] += error * 7.0 / 16.0;
                if (x > 0) errorMatrix[x - 1][y + 1] += error * 3.0 / 16.0;
                errorMatrix[x][y + 1] += error * 5.0 / 16.0;
                errorMatrix[x + 1][y + 1] += error * 1.0 / 16.0;
            }
        }
    }

    /******************* 2.Image Filtering *****************/
    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
        {
            for (std::size_t y = 0; y < input.GetSizeY(); ++y)
            {
                glm::vec3 sum = {0, 0, 0};
                for (int a = -1; a <= 1; ++a)
                {
                    for (int b = -1; b <= 1; ++b)
                    {
                        if (x + a < 0 || x + a >= input.GetSizeX() || y + b < 0 || y + b >= input.GetSizeY()) continue;
                        sum += input.At(x + a, y + b);
                    }
                }
                sum /= 9.0;
                output.At(x, y) = sum;
            }
        }
    }

    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        double Gx[3][3] = {
            { -1, 0, 1 },
            { -2, 0, 2 },
            { -1, 0, 1 }
        };
        double Gy[3][3] = {
            { 1, 2, 1 },
            { 0, 0, 0 },
            { -1, -2, -1 }
        };
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
        {
            for (std::size_t y = 0; y < input.GetSizeY(); ++y)
            {
                glm::vec3 sumX = {0, 0, 0};
                glm::vec3 sumY = {0, 0, 0};
                for (int a = -1; a <= 1; ++a)
                {
                    for (int b = -1; b <= 1; ++b)
                    {
                        if (x + a < 0 || x + a >= input.GetSizeX() || y + b < 0 || y + b >= input.GetSizeY()) continue;
                        sumX += input.At(x + a, y + b) * (float) Gx[a + 1][b + 1];
                        sumY += input.At(x + a, y + b) * (float) Gy[a + 1][b + 1];
                    }
                }
                glm::vec3 sum = glm::sqrt(sumX * sumX + sumY * sumY);
                output.At(x, y) = sum;
            }
        }
    }

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) {
        output             = inputBack;
        std::size_t width  = inputFront.GetSizeX();
        std::size_t height = inputFront.GetSizeY();
        glm::vec3 * g      = new glm::vec3[width * height];
        memset(g, 0, sizeof(glm::vec3) * width * height);
        // set boundary condition
        for (std::size_t y = 0; y < height; ++y) {
            // set boundary for (0, y), your code: g[y * width] = ?
            // set boundary for (width - 1, y), your code: g[y * width + width - 1] = ?
            g[y * width] = inputBack.At(offset.x, offset.y + y) - inputFront.At(0, y);
            g[y * width + width - 1] = inputBack.At(offset.x + width - 1, offset.y + y) - inputFront.At(width - 1, y);
        }
        for (std::size_t x = 0; x < width; ++x) {
            // set boundary for (x, 0), your code: g[x] = ?
            // set boundary for (x, height - 1), your code: g[(height - 1) * width + x] = ?
            g[x] = inputBack.At(offset.x + x, offset.y) - inputFront.At(x, 0);
            g[(height - 1) * width + x] = inputBack.At(offset.x + x, offset.y + height - 1) - inputFront.At(x, height - 1);
        }

        // Jacobi iteration, solve Ag = b
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
        delete[] g;
    }

    /******************* 4. Line Drawing *****************/
    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
        int dy = 2 * (p1.y - p0.y);
        int dx = 2 * (p1.x - p0.x);
        if (abs(dy) < abs(dx))
        {
            int x0, x1, y0, y1;
            if (p1.x < p0.x)
            {
                x0 = p1.x;
                x1 = p0.x;
                y0 = p1.y;
                y1 = p0.y;
                dx = -dx;
                dy = -dy;
            }
            else
            {
                x0 = p0.x;
                x1 = p1.x;
                y0 = p0.y;
                y1 = p1.y;
            }
            int x, y = y0;
            if (dy > 0)
            {
                int dydx = dy - dx, F = dy - dx / 2;
                for (x = x0; x <= x1; ++x)
                {
                    canvas.At(x, y) = color;
                    if (F < 0)
                        F += dy;
                    else
                    {
                        ++y;
                        F += dydx;
                    }
                }
            }
            else
            {
                dy = -dy;
                int dydx = dy - dx, F = dy - dx / 2;
                for (x = x0; x <= x1; ++x)
                {
                    canvas.At(x, y) = color;
                    if (F < 0)
                        F += dy;
                    else
                    {
                        --y;
                        F += dydx;
                    }
                }
            }
        }
        else
        {
            int x0, x1, y0, y1;
            if (p1.y < p0.y)
            {
                x0 = p1.x;
                x1 = p0.x;
                y0 = p1.y;
                y1 = p0.y;
                dx = -dx;
                dy = -dy;
            }
            else
            {
                x0 = p0.x;
                x1 = p1.x;
                y0 = p0.y;
                y1 = p1.y;
            }
            int x = x0, y;
            if (dx > 0)
            {
                int dxdy = dx - dy, F = dx - dy / 2;
                for (y = y0; y <= y1; ++y)
                {
                    canvas.At(x, y) = color;
                    if (F < 0)
                        F += dx;
                    else
                    {
                        ++x;
                        F += dxdy;
                    }
                }
            }
            else
            {
                dx = -dx;
                int dxdy = dx - dy, F = dx - dy / 2;
                for (y = y0; y <= y1; ++y)
                {
                    canvas.At(x, y) = color;
                    if (F < 0)
                        F += dx;
                    else
                    {
                        --x;
                        F += dxdy;
                    }
                }
            }
        }
    }
    /******************* 5. Triangle Drawing *****************/
    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        glm::ivec2 q0 = p0;
        glm::ivec2 q1 = p1;
        glm::ivec2 q2 = p2;
        // 排序
        if (q0.y > q1.y)
        {
            glm::ivec2 tmp = q0;
            q0 = q1;
            q1 = tmp;
        }
        if (q1.y > q2.y)
        {
            glm::ivec2 tmp = q1;
            q1 = q2;
            q2 = tmp;
        }
        if (q0.y > q1.y)
        {
            glm::ivec2 tmp = q0;
            q0 = q1;
            q1 = tmp;
        }
        double x_L = q0.x;
        double x_R = q0.x;
        if ((q2.x - q0.x) * (q1.y - q0.y) < (q2.y - q0.y) * (q1.x - q0.x))
        {
            double d_L = (q2.x - q0.x) / (double) (q2.y - q0.y);
            double d_R = (q1.x - q0.x) / (double) (q1.y - q0.y);
            for (int y = q0.y; y < q1.y; ++y)
            {
                for (int x = x_L; x <= x_R; ++x) canvas.At(x, y) = color;
                x_L += d_L;
                x_R += d_R;
            }
            for (int x = x_L; x <= x_R; ++x) canvas.At(x, q1.y) = color;
            d_R = (q1.x - q2.x) / (double) (q1.y - q2.y);
            for (int y = q1.y; y < q2.y; ++y)
            {
                for (int x = x_L; x <= x_R; ++x) canvas.At(x, y) = color;
                x_L += d_L;
                x_R += d_R;
            }
            canvas.At(q2.x, q2.y) = color;
        }
        else
        {
            double d_L = (q1.x - q0.x) / (double) (q1.y - q0.y);
            double d_R = (q2.x - q0.x) / (double) (q2.y - q0.y);
            for (int y = q0.y; y < q1.y; ++y)
            {
                for (int x = x_L; x <= x_R; ++x) canvas.At(x, y) = color;
                x_L += d_L;
                x_R += d_R;
            }
            for (int x = x_L; x <= x_R; ++x) canvas.At(x, q1.y) = color;
            d_L = (q1.x - q2.x) / (double) (q1.y - q2.y);
            for (int y = q1.y; y < q2.y; ++y)
            {
                for (int x = x_L; x <= x_R; ++x) canvas.At(x, y) = color;
                x_L += d_L;
                x_R += d_R;
            }
            canvas.At(q2.x, q2.y) = color;
        }
    }

    /******************* 6. Image Supersampling *****************/
    void Supersample(
        ImageRGB &       output,    // (320, 320)
        ImageRGB const & input,     // (2500, 2500)
        int              rate)  {
        std::size_t w = output.GetSizeX();
        std::size_t h = output.GetSizeY();
        std::size_t W = input.GetSizeX();
        std::size_t H = input.GetSizeY();
        float stepX = W * 1.0f / w;
        float stepY = H * 1.0f / h;
        for (std::size_t x = 0; x < w; ++x)
        {
            for (std::size_t y = 0; y < h; ++y)
            {
                glm::vec3 sum = {0, 0, 0};
                for (int a = 0; a < rate; ++a)
                {
                    for (int b = 0; b < rate; ++b)
                    {
                        float u = x * stepX + (a + 0.5f) * stepX / rate;
                        float v = y * stepY + (b + 0.5f) * stepY / rate;
                        std::size_t i = glm::floor(u);
                        std::size_t j = glm::floor(v);
                        std::size_t k = std::min(i + 1, W - 1);
                        std::size_t l = std::min(j + 1, H - 1);
                        glm::vec3 color = (1 - (u - i)) * (1 - (v - j)) * input.At(i, j)
                                        + (u - i) * (1 - (v - j)) * input.At(k, j)
                                        + (1 - (u - i)) * (v - j) * input.At(i, l)
                                        + (u - i) * (v - j) * input.At(k, l);
                        sum += color;
                    }
                }
                sum /= (rate * rate);
                output.At(x, y) = sum;
            }
        }
    }

    /******************* 7. Bezier Curve *****************/
    // Note: Please finish the function [DrawLine] before trying this part.
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
        if (points.size() == 1)
            return points[0];
        std::vector<glm::vec2> new_points;
        new_points.reserve(points.size() - 1);
        for (int i = 0; i < points.size() - 1; ++i)
            new_points.push_back(glm::vec2 {points[i].x * (1 - t) + points[i + 1].x * t, points[i].y * (1 - t) + points[i + 1].y * t});
        return CalculateBezierPoint(new_points, t);
    }
} // namespace VCX::Labs::Drawing2D