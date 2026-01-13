#include "Labs/5-Visualization/tasks.h"

#include <numbers>
#include <algorithm>
#include <string>
#include <array>
#include <cmath>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Visualization {

    // Protovis 顺序的属性名称: cyl, dsp, lbs, hp, acc, mpg, year
    static const std::array<std::string, 7> attributeNames = {
        "cylinders", "displacement", "weight", "horsepower", "acceleration", "mileage", "year"
    };

    // 属性单位
    static const std::array<std::string, 7> attributeUnits = {
        "", " sq in", " lbs", " hp", " sec", " mpg", ""
    };

    // 按照 Protovis 顺序获取 Car 的属性值
    // 顺序: cylinders(0), displacement(1), weight(2), horsepower(3), acceleration(4), mileage(5), year(6)
    static float GetCarAttribute(const Car& car, int index) {
        switch (index) {
            case 0: return static_cast<float>(car.cylinders);
            case 1: return car.displacement;
            case 2: return car.weight;
            case 3: return car.horsepower;
            case 4: return car.acceleration;
            case 5: return car.mileage;
            case 6: return static_cast<float>(car.year);
            default: return 0.0f;
        }
    }

    // steelblue: RGB(70, 130, 180) -> (0.275, 0.510, 0.706)
    // brown: RGB(165, 42, 42) -> (0.647, 0.165, 0.165)
    static glm::vec4 LerpColor(float t) {
        glm::vec3 steelblue(0.275f, 0.510f, 0.706f);
        glm::vec3 brown(0.647f, 0.165f, 0.165f);
        glm::vec3 color = glm::mix(steelblue, brown, glm::clamp(t, 0.0f, 1.0f));
        return glm::vec4(color, 1.0f);
    }

    struct CoordinateStates {
        std::vector<float> minValues;   // 每个属性的最小值
        std::vector<float> maxValues;   // 每个属性的最大值
        std::vector<float> filterMin;   // 刷选范围的最小值
        std::vector<float> filterMax;   // 刷选范围的最大值
        int                numAttrs;    // 属性数量
        int                activeAxis;  // 当前激活的轴（用于着色）
        
        // 拖动状态
        int                draggingAxis;   // 正在拖动的轴 (-1 表示没有)
        bool               isSelecting;    // 是否正在选择新范围
        bool               isDraggingBar;  // 是否正在拖动整个刷选条
        float              dragStartY;     // 拖动开始时的 Y 坐标
        float              barDragOffset;  // 拖动条时的偏移
        
        bool               initialized;

        CoordinateStates() : numAttrs(7), activeAxis(5), draggingAxis(-1), 
                             isSelecting(false), isDraggingBar(false), initialized(false) {}

        void Initialize(const std::vector<Car>& data) {
            if (initialized) return;
            initialized = true;

            numAttrs = 7;
            minValues.resize(numAttrs, std::numeric_limits<float>::max());
            maxValues.resize(numAttrs, std::numeric_limits<float>::lowest());
            filterMin.resize(numAttrs);
            filterMax.resize(numAttrs);

            // 计算每个属性的范围
            for (const auto& car : data) {
                for (int i = 0; i < numAttrs; ++i) {
                    float val = GetCarAttribute(car, i);
                    minValues[i] = std::min(minValues[i], val);
                    maxValues[i] = std::max(maxValues[i], val);
                }
            }

            for (int i = 0; i < numAttrs; ++i) {
                float fudge = 0.5f;
                minValues[i] = std::floor(minValues[i]) - fudge;
                maxValues[i] = std::ceil(maxValues[i]) + fudge;
                filterMin[i] = minValues[i];
                filterMax[i] = maxValues[i];
            }
        }

        // 将属性值归一化到 [0, 1]，0 在底部，1 在顶部
        float Normalize(int attrIndex, float value) const {
            if (maxValues[attrIndex] - minValues[attrIndex] < 1e-6f) return 0.5f;
            return (value - minValues[attrIndex]) / (maxValues[attrIndex] - minValues[attrIndex]);
        }

        // 反归一化
        float Denormalize(int attrIndex, float normalizedValue) const {
            return minValues[attrIndex] + normalizedValue * (maxValues[attrIndex] - minValues[attrIndex]);
        }

        // 检查一条数据是否在刷选范围内
        bool IsInFilter(const Car& car) const {
            for (int i = 0; i < numAttrs; ++i) {
                float val = GetCarAttribute(car, i);
                if (val < filterMin[i] || val > filterMax[i]) {
                    return false;
                }
            }
            return true;
        }

        // 获取轴的 X 坐标 (百分比)
        float GetAxisX(int axisIndex) const {
            float margin = 0.06f;
            float usableWidth = 1.0f - 2 * margin;
            return margin + usableWidth * axisIndex / (numAttrs - 1);
        }

        // 处理交互，返回是否有变化
        bool Update(const InteractProxy& proxy) {
            bool changed = false;
            glm::vec2 mousePos = proxy.MousePos();

            float topMargin = 0.12f;
            float bottomMargin = 0.08f;
            float axisTop = topMargin;
            float axisBottom = 1.0f - bottomMargin;

            // 鼠标拖动处理
            if (proxy.IsDragging()) {
                if (draggingAxis < 0) {
                    // 开始新的拖动 - 检查是否在某个轴附近
                    glm::vec2 startPos = proxy.DraggingStartPoint();
                    for (int i = 0; i < numAttrs; ++i) {
                        float axisX = GetAxisX(i);
                        if (std::abs(startPos.x - axisX) < 0.04f) {
                            float normalizedY = 1.0f - (startPos.y - axisTop) / (axisBottom - axisTop);
                            if (normalizedY >= 0.0f && normalizedY <= 1.0f) {
                                draggingAxis = i;
                                activeAxis = i;  // 更新激活的轴
                                
                                // 检查是否点击在现有刷选条上
                                float barMinY = Normalize(i, filterMin[i]);
                                float barMaxY = Normalize(i, filterMax[i]);
                                
                                // 如果刷选条很小（接近全范围），则开始新选择
                                if (barMaxY - barMinY > 0.95f) {
                                    isSelecting = true;
                                    isDraggingBar = false;
                                    dragStartY = normalizedY;
                                    filterMin[i] = Denormalize(i, normalizedY);
                                    filterMax[i] = Denormalize(i, normalizedY);
                                } else if (normalizedY >= barMinY && normalizedY <= barMaxY) {
                                    // 拖动整个条
                                    isDraggingBar = true;
                                    isSelecting = false;
                                    barDragOffset = normalizedY - (barMinY + barMaxY) / 2.0f;
                                } else {
                                    // 开始新的选择
                                    isSelecting = true;
                                    isDraggingBar = false;
                                    dragStartY = normalizedY;
                                    filterMin[i] = Denormalize(i, normalizedY);
                                    filterMax[i] = Denormalize(i, normalizedY);
                                }
                                changed = true;
                            }
                            break;
                        }
                    }
                } else {
                    // 继续拖动
                    float normalizedY = 1.0f - (mousePos.y - axisTop) / (axisBottom - axisTop);
                    normalizedY = glm::clamp(normalizedY, 0.0f, 1.0f);
                    
                    if (isDraggingBar) {
                        // 拖动整个刷选条
                        float barHeight = Normalize(draggingAxis, filterMax[draggingAxis]) - 
                                         Normalize(draggingAxis, filterMin[draggingAxis]);
                        float newCenter = normalizedY - barDragOffset;
                        float newMin = glm::clamp(newCenter - barHeight / 2.0f, 0.0f, 1.0f - barHeight);
                        float newMax = newMin + barHeight;
                        filterMin[draggingAxis] = Denormalize(draggingAxis, newMin);
                        filterMax[draggingAxis] = Denormalize(draggingAxis, newMax);
                    } else if (isSelecting) {
                        // 选择新范围
                        float value = Denormalize(draggingAxis, normalizedY);
                        float startValue = Denormalize(draggingAxis, dragStartY);
                        filterMin[draggingAxis] = std::min(value, startValue);
                        filterMax[draggingAxis] = std::max(value, startValue);
                    }
                    changed = true;
                }
            } else {
                // 释放拖动
                if (draggingAxis >= 0) {
                    // 如果刷选范围很小，重置为全范围
                    float range = filterMax[draggingAxis] - filterMin[draggingAxis];
                    float totalRange = maxValues[draggingAxis] - minValues[draggingAxis];
                    if (range < totalRange * 0.01f) {
                        filterMin[draggingAxis] = minValues[draggingAxis];
                        filterMax[draggingAxis] = maxValues[draggingAxis];
                    }
                    changed = true;
                }
                draggingAxis = -1;
                isSelecting = false;
                isDraggingBar = false;
            }

            // 右键双击重置刷选
            if (proxy.IsClicking(false)) {
                for (int i = 0; i < numAttrs; ++i) {
                    float axisX = GetAxisX(i);
                    if (std::abs(mousePos.x - axisX) < 0.04f) {
                        filterMin[i] = minValues[i];
                        filterMax[i] = maxValues[i];
                        activeAxis = i;
                        changed = true;
                        break;
                    }
                }
            }

            return changed;
        }

        void Paint(Common::ImageRGB& canvas, const std::vector<Car>& data) const {
            // 颜色定义
            glm::vec4 bgColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
            glm::vec4 axisColor = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
            glm::vec4 textColor = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
            glm::vec4 lineColorInactive = glm::vec4(0.87f, 0.87f, 0.87f, 1.0f);  // #ddd
            glm::vec4 inactiveBarColor = glm::vec4(0.5f, 0.5f, 0.5f, 0.5f);

            float topMargin = 0.12f;
            float bottomMargin = 0.08f;
            float axisTop = topMargin;
            float axisBottom = 1.0f - bottomMargin;

            // 设置背景
            SetBackGround(canvas, bgColor);

            // 首先绘制不在刷选范围内的线（灰色）
            for (const auto& car : data) {
                if (!IsInFilter(car)) {
                    for (int i = 0; i < numAttrs - 1; ++i) {
                        float x1 = GetAxisX(i);
                        float x2 = GetAxisX(i + 1);
                        // Y 轴反转：值大的在上面
                        float y1 = axisBottom - Normalize(i, GetCarAttribute(car, i)) * (axisBottom - axisTop);
                        float y2 = axisBottom - Normalize(i + 1, GetCarAttribute(car, i + 1)) * (axisBottom - axisTop);
                        DrawLine(canvas, lineColorInactive, glm::vec2(x1, y1), glm::vec2(x2, y2), 1.0f);
                    }
                }
            }

            // 然后绘制在刷选范围内的线（彩色 - 根据 active 轴的值着色）
            for (const auto& car : data) {
                if (IsInFilter(car)) {
                    float colorT = Normalize(activeAxis, GetCarAttribute(car, activeAxis));
                    glm::vec4 lineColor = LerpColor(colorT);
                    
                    for (int i = 0; i < numAttrs - 1; ++i) {
                        float x1 = GetAxisX(i);
                        float x2 = GetAxisX(i + 1);
                        float y1 = axisBottom - Normalize(i, GetCarAttribute(car, i)) * (axisBottom - axisTop);
                        float y2 = axisBottom - Normalize(i + 1, GetCarAttribute(car, i + 1)) * (axisBottom - axisTop);
                        DrawLine(canvas, lineColor, glm::vec2(x1, y1), glm::vec2(x2, y2), 1.0f);
                    }
                }
            }

            // 绘制坐标轴和标签
            for (int i = 0; i < numAttrs; ++i) {
                float x = GetAxisX(i);
                DrawLine(canvas, axisColor, glm::vec2(x, axisTop), glm::vec2(x, axisBottom), 1.0f);

                // 绘制属性名称（顶部，增加距离避免与数值标签重叠）
                PrintText(canvas, textColor, glm::vec2(x, axisTop - 0.06f), 0.025f, attributeNames[i]);

                // 绘制刷选范围指示器
                float barMinNorm = Normalize(i, filterMin[i]);
                float barMaxNorm = Normalize(i, filterMax[i]);
                float barY1 = axisBottom - barMaxNorm * (axisBottom - axisTop);  // max 在上
                float barY2 = axisBottom - barMinNorm * (axisBottom - axisTop);  // min 在下
                float barWidth = 0.012f;

                // 刷选条颜色
                glm::vec4 barColor;
                if (i == activeAxis) {
                    float midValue = (filterMin[i] + filterMax[i]) / 2.0f;
                    float colorT = Normalize(i, midValue);
                    barColor = LerpColor(colorT);
                } else {
                    barColor = inactiveBarColor;
                }

                // 绘制刷选条
                DrawFilledRect(canvas, barColor, 
                               glm::vec2(x - barWidth / 2, barY1), 
                               glm::vec2(barWidth, barY2 - barY1));
                
                // 绘制刷选条边框
                glm::vec4 borderColor(1.0f, 1.0f, 1.0f, 1.0f);
                DrawRect(canvas, borderColor, 
                         glm::vec2(x - barWidth / 2, barY1), 
                         glm::vec2(barWidth, barY2 - barY1), 1.0f);

                // 绘制范围数值标签
                char buffer[32];
                snprintf(buffer, sizeof(buffer), "%.0f%s", filterMax[i], attributeUnits[i].c_str());
                PrintText(canvas, textColor, glm::vec2(x, barY1 - 0.02f), 0.018f, buffer);
                snprintf(buffer, sizeof(buffer), "%.0f%s", filterMin[i], attributeUnits[i].c_str());
                PrintText(canvas, textColor, glm::vec2(x, barY2 + 0.025f), 0.018f, buffer);
            }
        }
    };

    bool PaintParallelCoordinates(Common::ImageRGB & input, InteractProxy const & proxy, std::vector<Car> const & data, bool force) {
        static CoordinateStates states;
        states.Initialize(data);

        bool change = states.Update(proxy);
        if (!force && !change) return false;

        states.Paint(input, data);
        return true;
    }

    void LIC(ImageRGB & output, Common::ImageRGB const & noise, VectorField2D const & field, int const & step) {
        uint32_t width = field.size.first;
        uint32_t height = field.size.second;

        // 确保输出大小正确
        if (output.GetSizeX() != width || output.GetSizeY() != height) {
            output = ImageRGB(width, height);
        }

        // 对每个像素进行 LIC 计算
        for (uint32_t y = 0; y < height; ++y) {
            for (uint32_t x = 0; x < width; ++x) {
                float sumR = 0.0f, sumG = 0.0f, sumB = 0.0f;
                float weight = 0.0f;

                // 正向追踪流线
                float px = static_cast<float>(x);
                float py = static_cast<float>(y);
                for (int s = 0; s < step; ++s) {
                    int ix = static_cast<int>(std::round(px));
                    int iy = static_cast<int>(std::round(py));
                    if (ix < 0 || ix >= static_cast<int>(width) || iy < 0 || iy >= static_cast<int>(height)) break;

                    glm::vec3 noiseColor = noise.At(static_cast<uint32_t>(ix), static_cast<uint32_t>(iy));
                    float k = 1.0f; // 可以使用高斯核，这里使用均匀权重
                    sumR += k * noiseColor.r;
                    sumG += k * noiseColor.g;
                    sumB += k * noiseColor.b;
                    weight += k;

                    // 获取当前位置的向量场
                    glm::vec2 vec = field.At(static_cast<uint32_t>(ix), static_cast<uint32_t>(iy));
                    float len = glm::length(vec);
                    if (len < 1e-6f) break;
                    vec = vec / len; // 归一化

                    px += vec.x;
                    py += vec.y;
                }

                // 反向追踪流线
                px = static_cast<float>(x);
                py = static_cast<float>(y);
                for (int s = 0; s < step; ++s) {
                    // 获取当前位置的向量场
                    int ix = static_cast<int>(std::round(px));
                    int iy = static_cast<int>(std::round(py));
                    if (ix < 0 || ix >= static_cast<int>(width) || iy < 0 || iy >= static_cast<int>(height)) break;

                    glm::vec2 vec = field.At(static_cast<uint32_t>(ix), static_cast<uint32_t>(iy));
                    float len = glm::length(vec);
                    if (len < 1e-6f) break;
                    vec = vec / len; // 归一化

                    px -= vec.x;
                    py -= vec.y;

                    ix = static_cast<int>(std::round(px));
                    iy = static_cast<int>(std::round(py));
                    if (ix < 0 || ix >= static_cast<int>(width) || iy < 0 || iy >= static_cast<int>(height)) break;

                    glm::vec3 noiseColor = noise.At(static_cast<uint32_t>(ix), static_cast<uint32_t>(iy));
                    float k = 1.0f;
                    sumR += k * noiseColor.r;
                    sumG += k * noiseColor.g;
                    sumB += k * noiseColor.b;
                    weight += k;
                }

                // 计算平均值
                if (weight > 0) {
                    output.At(x, y) = glm::vec3(sumR / weight, sumG / weight, sumB / weight);
                } else {
                    output.At(x, y) = noise.At(x, y);
                }
            }
        }
    }
}; // namespace VCX::Labs::Visualization