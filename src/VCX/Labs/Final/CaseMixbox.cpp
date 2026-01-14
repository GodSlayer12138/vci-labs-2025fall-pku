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
        // Return selected palette color
        return glm::vec3(
            _palette[_selectedColorIndex][0] / 255.0f,
            _palette[_selectedColorIndex][1] / 255.0f,
            _palette[_selectedColorIndex][2] / 255.0f
        );
    }

    void CaseMixbox::DrawBrushStroke(int cx, int cy, float dirX, float dirY) {
        int halfWidth = _brushSize;
        int thickness = static_cast<int>(_brushSize / 3); // Thickness in movement direction (rectangle depth)
        
        // Adjustable opacity for accumulative painting (base alpha 0.05)
        const float baseAlpha = 0.05f;
        const float alpha = baseAlpha * (_brushOpacity / 100.0f);
        
        // Normalize direction vector
        float len = std::sqrt(dirX * dirX + dirY * dirY);
        if (len < 0.001f) {
            // Default direction if no movement
            dirX = 1.0f;
            dirY = 0.0f;
            len = 1.0f;
        }
        dirX /= len;
        dirY /= len;
        
        // Perpendicular direction (rotate 90 degrees)
        float perpX = -dirY;
        float perpY = dirX;
        
        // Draw rectangular brush: width perpendicular to movement, thickness along movement
        for (int j = -thickness; j <= thickness; ++j) {
            for (int i = -halfWidth; i <= halfWidth; ++i) {
                int x = cx + static_cast<int>(i * perpX + j * dirX);
                int y = cy + static_cast<int>(i * perpY + j * dirY);
                
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
        ImGui::Text("Color Palette");
        ImGui::Separator();
        
        // 4x4 color grid
        const float buttonSize = 50.0f;
        const float spacing = 4.0f;
        
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                int index = row * 4 + col;
                
                if (col > 0) ImGui::SameLine(0.0f, spacing);
                
                ImGui::PushID(index);
                
                // Get color
                ImVec4 color(_palette[index][0] / 255.0f,
                            _palette[index][1] / 255.0f,
                            _palette[index][2] / 255.0f,
                            1.0f);
                
                // Draw border if selected
                if (_selectedColorIndex == index) {
                    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 3.0f);
                }
                
                // Color button
                if (ImGui::ColorButton("##colorBtn", color, 
                                      ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoBorder,
                                      ImVec2(buttonSize, buttonSize))) {
                    _selectedColorIndex = index;
                }
                
                if (_selectedColorIndex == index) {
                    ImGui::PopStyleVar();
                    ImGui::PopStyleColor();
                }
                
                // Right-click for custom colors (last 3 slots)
                if (index >= 13 && ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                    ImGui::OpenPopup("customColorPicker");
                }
                
                // Custom color picker popup
                if (index >= 13 && ImGui::BeginPopup("customColorPicker")) {
                    float tempColor[3] = {
                        _palette[index][0] / 255.0f,
                        _palette[index][1] / 255.0f,
                        _palette[index][2] / 255.0f
                    };
                    if (ImGui::ColorPicker3("##picker", tempColor,
                                           ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_InputRGB)) {
                        _palette[index][0] = static_cast<int>(std::round(tempColor[0] * 255.0f));
                        _palette[index][1] = static_cast<int>(std::round(tempColor[1] * 255.0f));
                        _palette[index][2] = static_cast<int>(std::round(tempColor[2] * 255.0f));
                    }
                    if (ImGui::Button("Close")) ImGui::CloseCurrentPopup();
                    ImGui::EndPopup();
                }
                
                ImGui::PopID();
            }
        }
        
        ImGui::Spacing();
        ImGui::Text("Current Color:");
        glm::vec3 currentColor = GetCurrentBrushColor();
        int r = static_cast<int>(currentColor.r * 255);
        int g = static_cast<int>(currentColor.g * 255);
        int b = static_cast<int>(currentColor.b * 255);
        ImGui::ColorButton("##current", ImVec4(currentColor.r, currentColor.g, currentColor.b, 1.0f),
                          ImGuiColorEditFlags_None, ImVec2(80, 30));
        ImGui::SameLine();
        ImGui::Text("(%d, %d, %d)", r, g, b);
        ImGui::Text("Right-click last 3 to customize");
        
        ImGui::Spacing();
        ImGui::Text("Color Mixer");
        ImGui::Separator();
        
        // Color 1 selection
        ImGui::Text("Color 1:");
        ImVec4 mixColor1(_palette[_mixerColor1Index][0] / 255.0f,
                        _palette[_mixerColor1Index][1] / 255.0f,
                        _palette[_mixerColor1Index][2] / 255.0f, 1.0f);
        if (ImGui::ColorButton("##mixColor1", mixColor1, ImGuiColorEditFlags_None, ImVec2(40, 40))) {
            ImGui::OpenPopup("SelectMixColor1");
        }
        ImGui::SameLine();
        ImGui::Text("(%d, %d, %d)", _palette[_mixerColor1Index][0], 
                    _palette[_mixerColor1Index][1], _palette[_mixerColor1Index][2]);
        
        // Popup for selecting color 1 from palette
        if (ImGui::BeginPopup("SelectMixColor1")) {
            ImGui::Text("Select Color 1");
            const float smallButtonSize = 35.0f;
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    int index = row * 4 + col;
                    if (col > 0) ImGui::SameLine();
                    ImGui::PushID(index);
                    ImVec4 c(_palette[index][0] / 255.0f,
                            _palette[index][1] / 255.0f,
                            _palette[index][2] / 255.0f, 1.0f);
                    if (ImGui::ColorButton("##sel", c, ImGuiColorEditFlags_None, 
                                          ImVec2(smallButtonSize, smallButtonSize))) {
                        _mixerColor1Index = index;
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::PopID();
                }
            }
            ImGui::EndPopup();
        }
        
        // Color 2 selection
        ImGui::Text("Color 2:");
        ImVec4 mixColor2(_palette[_mixerColor2Index][0] / 255.0f,
                        _palette[_mixerColor2Index][1] / 255.0f,
                        _palette[_mixerColor2Index][2] / 255.0f, 1.0f);
        if (ImGui::ColorButton("##mixColor2", mixColor2, ImGuiColorEditFlags_None, ImVec2(40, 40))) {
            ImGui::OpenPopup("SelectMixColor2");
        }
        ImGui::SameLine();
        ImGui::Text("(%d, %d, %d)", _palette[_mixerColor2Index][0], 
                    _palette[_mixerColor2Index][1], _palette[_mixerColor2Index][2]);
        
        // Popup for selecting color 2 from palette
        if (ImGui::BeginPopup("SelectMixColor2")) {
            ImGui::Text("Select Color 2");
            const float smallButtonSize = 35.0f;
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    int index = row * 4 + col;
                    if (col > 0) ImGui::SameLine();
                    ImGui::PushID(index);
                    ImVec4 c(_palette[index][0] / 255.0f,
                            _palette[index][1] / 255.0f,
                            _palette[index][2] / 255.0f, 1.0f);
                    if (ImGui::ColorButton("##sel", c, ImGuiColorEditFlags_None, 
                                          ImVec2(smallButtonSize, smallButtonSize))) {
                        _mixerColor2Index = index;
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::PopID();
                }
            }
            ImGui::EndPopup();
        }
        
        // Mix ratio slider
        ImGui::SliderFloat("Mix Ratio", &_mixRatio, 0.0f, 1.0f, "%.2f");
        
        // Show mixed result
        unsigned char mixedR, mixedG, mixedB;
        mixbox_lerp(_palette[_mixerColor1Index][0], _palette[_mixerColor1Index][1], _palette[_mixerColor1Index][2],
                   _palette[_mixerColor2Index][0], _palette[_mixerColor2Index][1], _palette[_mixerColor2Index][2],
                   _mixRatio,
                   &mixedR, &mixedG, &mixedB);
        ImGui::Text("Mixed Result:");
        ImVec4 mixedColor(mixedR / 255.0f, mixedG / 255.0f, mixedB / 255.0f, 1.0f);
        ImGui::ColorButton("##mixedResult", mixedColor, ImGuiColorEditFlags_None, ImVec2(60, 40));
        ImGui::SameLine();
        ImGui::Text("(%d, %d, %d)", mixedR, mixedG, mixedB);
        if (ImGui::Button("Apply to Brush")) {
            // Find an empty custom slot or use the last one
            int targetIndex = 15; // Default to last custom slot
            for (int i = 13; i < 16; ++i) {
                if (_palette[i][0] == 255 && _palette[i][1] == 255 && _palette[i][2] == 255) {
                    targetIndex = i;
                    break;
                }
            }
            _palette[targetIndex][0] = mixedR;
            _palette[targetIndex][1] = mixedG;
            _palette[targetIndex][2] = mixedB;
            _selectedColorIndex = targetIndex;
        }
        
        ImGui::Spacing();
        ImGui::Text("Brush Settings");
        ImGui::Separator();
        ImGui::SliderInt("Brush Size", &_brushSize, 20, 50);
        ImGui::SliderInt("Brush Opacity", &_brushOpacity, 0, 100, "%d%%");
        
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
                // Only draw after the first click when mouse moves
                bool shouldDraw = false;
                
                if (!_isDrawing) {
                    // First click - just record position, don't draw
                    _isDrawing = true;
                    _lastDrawPos = ImVec2(static_cast<float>(x), static_cast<float>(y));
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
                    
                    // Calculate movement direction
                    float dirX = 1.0f;
                    float dirY = 0.0f;
                    if (_isDrawing && _lastDrawPos.x >= 0) {
                        dirX = x - _lastDrawPos.x;
                        dirY = y - _lastDrawPos.y;
                    }
                    
                    if (_isDrawing && _lastDrawPos.x >= 0 && (_lastDrawPos.x != x || _lastDrawPos.y != y)) {
                        // Interpolate between last position and current for smooth strokes
                        float lastX = _lastDrawPos.x;
                        float lastY = _lastDrawPos.y;
                        float dist = std::sqrt((x - lastX) * (x - lastX) + (y - lastY) * (y - lastY));
                        // Use smaller step size for denser brush strokes
                        float stepSize = std::max(0.5f, static_cast<float>(_brushSize) * 0.15f);
                        int steps = std::max(1, static_cast<int>(dist / stepSize));
                        
                        for (int i = 1; i <= steps; ++i) {
                            float t = static_cast<float>(i) / steps;
                            int interpX = static_cast<int>(lastX + t * (x - lastX));
                            int interpY = static_cast<int>(lastY + t * (y - lastY));
                            DrawBrushStroke(interpX, interpY, dirX, dirY);
                        }
                    } else {
                        DrawBrushStroke(x, y, dirX, dirY);
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

