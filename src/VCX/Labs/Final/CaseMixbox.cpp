#include <algorithm>
#include <cmath>

#include "Labs/Final/CaseMixbox.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Final {

    CaseMixbox::CaseMixbox() :
        _texture(Engine::GL::SamplerOptions {
            .MinFilter = Engine::GL::FilterMode::Linear,
            .MagFilter = Engine::GL::FilterMode::Nearest
        }) {
        ClearCanvas();
    }

    void CaseMixbox::ClearCanvas() {
        _canvas = Common::CreatePureImageRGB(_canvasWidth, _canvasHeight, glm::vec3(1.0f, 1.0f, 1.0f));
        _canvasDirty = true;
    }

    glm::vec3 CaseMixbox::GetCurrentBrushColor() const {
        // Mixbox pigment-based mixing
        unsigned char r, g, b;
        mixbox_lerp(_color1[0], _color1[1], _color1[2],
                   _color2[0], _color2[1], _color2[2],
                   _mixRatio,
                   &r, &g, &b);
        return glm::vec3(r / 255.0f, g / 255.0f, b / 255.0f);
    }

    void CaseMixbox::DrawBrushStroke(int cx, int cy) {
        int radius = _brushSize;
        int radiusSq = radius * radius;
        
        // Fixed low opacity for accumulative painting
        constexpr float alpha = 0.03f;
        
        for (int dx = -radius; dx <= radius; ++dx) {
            int dxSq = dx * dx;
            for (int dy = -radius; dy <= radius; ++dy) {
                // Use squared distance to avoid sqrt
                int distSq = dxSq + dy * dy;
                if (distSq > radiusSq) continue;
                
                int x = cx + dx;
                int y = cy + dy;
                
                // Check bounds
                if (x < 0 || x >= _canvasWidth || y < 0 || y >= _canvasHeight) continue;
                
                // Get existing color
                glm::vec3 existing = _canvas.At(x, y);
                
                // Use mixbox for color blending to simulate paint mixing
                unsigned char existingR = static_cast<unsigned char>(existing.r * 255.0f);
                unsigned char existingG = static_cast<unsigned char>(existing.g * 255.0f);
                unsigned char existingB = static_cast<unsigned char>(existing.b * 255.0f);
                
                unsigned char blendedR, blendedG, blendedB;
                mixbox_lerp(existingR, existingG, existingB,
                           _cachedBrushR, _cachedBrushG, _cachedBrushB,
                           alpha,
                           &blendedR, &blendedG, &blendedB);
                
                _canvas.At(x, y) = glm::vec3(blendedR / 255.0f, blendedG / 255.0f, blendedB / 255.0f);
            }
        }
        _canvasDirty = true;
    }

    void CaseMixbox::OnSetupPropsUI() {
        ImGui::Text("Colors to Mix (0-255)");
        ImGui::Separator();
        
        // Color 1 with 0-255 sliders
        ImGui::Text("Color 1:");
        ImGui::SliderInt("R##1", &_color1[0], 0, 255);
        ImGui::SliderInt("G##1", &_color1[1], 0, 255);
        ImGui::SliderInt("B##1", &_color1[2], 0, 255);
        ImGui::ColorButton("##color1", ImVec4(_color1[0]/255.0f, _color1[1]/255.0f, _color1[2]/255.0f, 1.0f),
                          ImGuiColorEditFlags_None, ImVec2(40, 20));
        ImGui::SameLine();
        ImGui::Text("(%d, %d, %d)", _color1[0], _color1[1], _color1[2]);
        ImGui::SameLine();
        if (ImGui::SmallButton("Picker##1")) {
            ImGui::OpenPopup("ColorPicker1");
        }
        if (ImGui::BeginPopup("ColorPicker1")) {
            float c1[3] = { _color1[0] / 255.0f, _color1[1] / 255.0f, _color1[2] / 255.0f };
            if (ImGui::ColorPicker3("##picker1", c1, ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_InputRGB)) {
                _color1[0] = static_cast<int>(std::round(c1[0] * 255.0f));
                _color1[1] = static_cast<int>(std::round(c1[1] * 255.0f));
                _color1[2] = static_cast<int>(std::round(c1[2] * 255.0f));
            }
            if (ImGui::Button("Close##p1")) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
        
        ImGui::Spacing();
        
        // Color 2 with 0-255 sliders
        ImGui::Text("Color 2:");
        ImGui::SliderInt("R##2", &_color2[0], 0, 255);
        ImGui::SliderInt("G##2", &_color2[1], 0, 255);
        ImGui::SliderInt("B##2", &_color2[2], 0, 255);
        ImGui::ColorButton("##color2", ImVec4(_color2[0]/255.0f, _color2[1]/255.0f, _color2[2]/255.0f, 1.0f),
                          ImGuiColorEditFlags_None, ImVec2(40, 20));
        ImGui::SameLine();
        ImGui::Text("(%d, %d, %d)", _color2[0], _color2[1], _color2[2]);
        ImGui::SameLine();
        if (ImGui::SmallButton("Picker##2")) {
            ImGui::OpenPopup("ColorPicker2");
        }
        if (ImGui::BeginPopup("ColorPicker2")) {
            float c2[3] = { _color2[0] / 255.0f, _color2[1] / 255.0f, _color2[2] / 255.0f };
            if (ImGui::ColorPicker3("##picker2", c2, ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_InputRGB)) {
                _color2[0] = static_cast<int>(std::round(c2[0] * 255.0f));
                _color2[1] = static_cast<int>(std::round(c2[1] * 255.0f));
                _color2[2] = static_cast<int>(std::round(c2[2] * 255.0f));
            }
            if (ImGui::Button("Close##p2")) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
        
        ImGui::Spacing();
        ImGui::SliderFloat("Mix Ratio", &_mixRatio, 0.0f, 1.0f, "%.2f");
        
        ImGui::Spacing();
        ImGui::Text("Mixed Color:");
        glm::vec3 mixedColor = GetCurrentBrushColor();
        int mixedR = static_cast<int>(mixedColor.r * 255);
        int mixedG = static_cast<int>(mixedColor.g * 255);
        int mixedB = static_cast<int>(mixedColor.b * 255);
        ImGui::ColorButton("##mixed", ImVec4(mixedColor.r, mixedColor.g, mixedColor.b, 1.0f), 
                           ImGuiColorEditFlags_None, ImVec2(80, 30));
        ImGui::SameLine();
        ImGui::Text("(%d, %d, %d)", mixedR, mixedG, mixedB);
        
        ImGui::Spacing();
        ImGui::Text("Brush Settings");
        ImGui::Separator();
        ImGui::SliderInt("Brush Size", &_brushSize, 1, 50);
        
        ImGui::Spacing();
        if (ImGui::Button("Clear Canvas")) {
            ClearCanvas();
        }
    }

    Common::CaseRenderResult CaseMixbox::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_canvasDirty) {
            _texture.Update(_canvas);
            _canvasDirty = false;
        }
        
        return Common::CaseRenderResult {
            .Fixed     = true,
            .Image     = _texture,
            .ImageSize = { static_cast<std::uint32_t>(_canvasWidth), 
                           static_cast<std::uint32_t>(_canvasHeight) },
        };
    }

    void CaseMixbox::OnProcessInput(ImVec2 const & pos) {
        auto         window  = ImGui::GetCurrentWindow();
        bool         hovered = false;
        bool         anyHeld = false;
        ImVec2 const delta   = ImGui::GetIO().MouseDelta;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hovered, &anyHeld);
        
        if (!hovered) {
            _isDrawing = false;
            _lastDrawPos = ImVec2(-1, -1);
            return;
        }
        
        // pos is already in image-relative coordinates
        int x = static_cast<int>(pos.x);
        int y = static_cast<int>(pos.y);
        
        // Handle drawing
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            if (x >= 0 && x < _canvasWidth && y >= 0 && y < _canvasHeight) {
                // Only draw if mouse has moved or this is the first click
                bool shouldDraw = false;
                
                if (!_isDrawing) {
                    // First click - draw once
                    shouldDraw = true;
                    _isDrawing = true;
                } else if (_lastDrawPos.x >= 0) {
                    // Check if mouse has moved
                    float dx = x - _lastDrawPos.x;
                    float dy = y - _lastDrawPos.y;
                    float distMoved = std::sqrt(dx * dx + dy * dy);
                    
                    // Only draw if moved at least 1 pixel
                    if (distMoved >= 1.0f) {
                        shouldDraw = true;
                    }
                }
                
                if (shouldDraw) {
                    // Update cached brush color before drawing
                    _cachedBrushColor = GetCurrentBrushColor();
                    _cachedBrushR = static_cast<unsigned char>(_cachedBrushColor.r * 255.0f);
                    _cachedBrushG = static_cast<unsigned char>(_cachedBrushColor.g * 255.0f);
                    _cachedBrushB = static_cast<unsigned char>(_cachedBrushColor.b * 255.0f);
                    
                    if (_isDrawing && _lastDrawPos.x >= 0 && _lastDrawPos.x != x && _lastDrawPos.y != y) {
                        // Interpolate between last position and current for smooth strokes
                        float lastX = _lastDrawPos.x;
                        float lastY = _lastDrawPos.y;
                        float dist = std::sqrt((x - lastX) * (x - lastX) + (y - lastY) * (y - lastY));
                        // Use larger step size (brush radius / 2) to reduce overlapping draws
                        float stepSize = std::max(1.0f, static_cast<float>(_brushSize) * 0.5f);
                        int steps = std::max(1, static_cast<int>(dist / stepSize));
                        
                        for (int i = 1; i <= steps; ++i) {
                            float t = static_cast<float>(i) / steps;
                            int interpX = static_cast<int>(lastX + t * (x - lastX));
                            int interpY = static_cast<int>(lastY + t * (y - lastY));
                            DrawBrushStroke(interpX, interpY);
                        }
                    } else {
                        DrawBrushStroke(x, y);
                    }
                    
                    _lastDrawPos = ImVec2(static_cast<float>(x), static_cast<float>(y));
                }
            }
        } else {
            _isDrawing = false;
            _lastDrawPos = ImVec2(-1, -1);
        }
        
        // Handle panning with middle mouse or right mouse
        if (ImGui::IsMouseDown(ImGuiMouseButton_Middle) || ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
            if (delta.x != 0.f)
                ImGui::SetScrollX(window, window->Scroll.x - delta.x);
            if (delta.y != 0.f)
                ImGui::SetScrollY(window, window->Scroll.y - delta.y);
        }
    }

} // namespace VCX::Labs::Final

