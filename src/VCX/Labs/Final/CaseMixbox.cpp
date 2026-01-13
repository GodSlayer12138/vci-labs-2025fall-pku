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
    }

    glm::vec3 CaseMixbox::GetCurrentBrushColor() const {
        if (_useRgbMixing) {
            // Standard RGB linear interpolation (0-255 range)
            int r = static_cast<int>(_color1[0] * (1.0f - _mixRatio) + _color2[0] * _mixRatio);
            int g = static_cast<int>(_color1[1] * (1.0f - _mixRatio) + _color2[1] * _mixRatio);
            int b = static_cast<int>(_color1[2] * (1.0f - _mixRatio) + _color2[2] * _mixRatio);
            return glm::vec3(r / 255.0f, g / 255.0f, b / 255.0f);
        } else {
            // Mixbox pigment-based mixing
            unsigned char r, g, b;
            mixbox_lerp(_color1[0], _color1[1], _color1[2],
                       _color2[0], _color2[1], _color2[2],
                       _mixRatio,
                       &r, &g, &b);
            return glm::vec3(r / 255.0f, g / 255.0f, b / 255.0f);
        }
    }

    void CaseMixbox::DrawBrushStroke(int cx, int cy) {
        glm::vec3 brushColor = GetCurrentBrushColor();
        
        int radius = _brushSize;
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                int x = cx + dx;
                int y = cy + dy;
                
                // Check bounds
                if (x < 0 || x >= _canvasWidth || y < 0 || y >= _canvasHeight) continue;
                
                // Circular brush with soft edges
                float dist = std::sqrt(static_cast<float>(dx * dx + dy * dy));
                if (dist > radius) continue;
                
                // Soft edge falloff
                float alpha = 1.0f - (dist / radius);
                alpha = alpha * alpha; // Quadratic falloff for softer edges
                alpha *= 0.8f; // Increase opacity for more visible strokes
                
                // Blend with existing color
                glm::vec3 existing = _canvas.At(x, y);
                glm::vec3 blended = glm::mix(existing, brushColor, alpha);
                _canvas.At(x, y) = blended;
            }
        }
    }

    void CaseMixbox::OnSetupPropsUI() {
        ImGui::Text("Color Mixing Mode");
        ImGui::Separator();
        
        if (ImGui::RadioButton("Mixbox (Pigment)", !_useRgbMixing)) {
            _useRgbMixing = false;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Standard RGB", _useRgbMixing)) {
            _useRgbMixing = true;
        }
        
        ImGui::Spacing();
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
        
        ImGui::Spacing();
        ImGui::Checkbox("Zoom Tooltip", &_enableZoom);
    }

    Common::CaseRenderResult CaseMixbox::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        _texture.Update(_canvas);
        
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
                if (_isDrawing && _lastDrawPos.x >= 0) {
                    // Interpolate between last position and current for smooth strokes
                    float lastX = _lastDrawPos.x;
                    float lastY = _lastDrawPos.y;
                    float dist = std::sqrt((x - lastX) * (x - lastX) + (y - lastY) * (y - lastY));
                    int steps = std::max(1, static_cast<int>(dist / 2.0f));
                    
                    for (int i = 0; i <= steps; ++i) {
                        float t = static_cast<float>(i) / steps;
                        int interpX = static_cast<int>(lastX + t * (x - lastX));
                        int interpY = static_cast<int>(lastY + t * (y - lastY));
                        DrawBrushStroke(interpX, interpY);
                    }
                } else {
                    DrawBrushStroke(x, y);
                }
                _isDrawing = true;
                _lastDrawPos = ImVec2(static_cast<float>(x), static_cast<float>(y));
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
        
        // Zoom tooltip
        if (_enableZoom && !anyHeld && ImGui::IsItemHovered() && !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            Common::ImGuiHelper::ZoomTooltip(_texture, 
                { static_cast<std::uint32_t>(_canvasWidth), static_cast<std::uint32_t>(_canvasHeight) }, 
                pos);
        }
    }

} // namespace VCX::Labs::Final
