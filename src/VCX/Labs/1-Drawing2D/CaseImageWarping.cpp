#include <algorithm>
#include <array>

#include "Engine/loader.h"
#include "Labs/1-Drawing2D/CaseImageWarping.h"
#include "Labs/1-Drawing2D/tasks.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Drawing2D {

    static constexpr auto c_Size = std::pair(180U, 215U);
    static constexpr std::array<glm::ivec2, 16> c_ControlPoints {
        glm::ivec2 {  0,   0},
        glm::ivec2 { 60,   0},
        glm::ivec2 {119,   0},
        glm::ivec2 {179,   0},
        glm::ivec2 {  0,  71},
        glm::ivec2 { 60,  71},
        glm::ivec2 {119,  71},
        glm::ivec2 {179,  71},
        glm::ivec2 {  0, 143},
        glm::ivec2 { 60, 143},
        glm::ivec2 {119, 143},
        glm::ivec2 {179, 143},
        glm::ivec2 {  0, 214},
        glm::ivec2 { 60, 214},
        glm::ivec2 {119, 214},
        glm::ivec2 {179, 214}
    };
    static constexpr glm::ivec2 c_ControlPointBound {10, 12};

    static void DrawPoint(Common::ImageRGB & canvas, glm::vec3 color, glm::ivec2 pos) {
        for (int dx = -2; dx <= 2; ++dx) {
            for (int dy = -2; dy <= 2; ++dy) {
                int x = pos.x + dx;
                int y = pos.y + dy;
                if (std::size_t(x) < canvas.GetSizeX() && std::size_t(y) < canvas.GetSizeY())
                    canvas.At(x, y) = color;
            }
        }
    }

    CaseImageWarping::CaseImageWarping():
        _texture({ .MinFilter = Engine::GL::FilterMode::Linear, .MagFilter = Engine::GL::FilterMode::Nearest }),
        _input(Common::AlphaBlend(
            Engine::LoadImageRGBA("assets/images/david-180x215.png"),
            Common::CreatePureImageRGB(c_Size.first, c_Size.second, { 0, 0, 0 }))),
        _empty(Common::CreateCheckboardImageRGB(c_Size.first, c_Size.second)),
        _vertices(c_ControlPoints) {
    }

    void CaseImageWarping::OnSetupPropsUI() {
        ImGui::Checkbox("Zoom Tooltip", &_enableZoom);
        ImGui::Checkbox("Enable Left-Button-Drag", &_enableLeftDrag);
        ImGui::TextWrapped(
            _enableLeftDrag ? 
            "Hint: use the left mouse button to drag the point." :
            "Hint: use the right mouse button to drag the point."
        );
        _recompute |= ImGui::Checkbox("Show Control Points", &_showControlPoints);
        _recompute |= ImGui::Checkbox("Show Grids", &_showGrids);
        _recompute |= ImGui::Checkbox("Show Image", &_showImage);
        if (ImGui::Button("Reset")) {
            _recompute = true;
            _vertices = c_ControlPoints;
        }
        Common::ImGuiHelper::SaveImage(_texture, c_Size);
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseImageWarping::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_recompute) {
            _recompute = false;
            Common::ImageRGB tex(c_Size.first, c_Size.second);
            if (_showImage)
                WarpImage(tex, _vertices, _input, c_ControlPoints);
            else
                tex = _empty;
            if (_showGrids) {
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        if (j > 0)
                            DrawLine(tex, { 0.3, 0.8, 0.5 }, _vertices[i * 4 + j], _vertices[i * 4 + j - 1]);
                        if (i > 0)
                            DrawLine(tex, { 0.3, 0.8, 0.5 }, _vertices[i * 4 + j], _vertices[i * 4 + j - 4]);
                        if (j > 0 && i < 3)
                            DrawLine(tex, { 0.3, 0.8, 0.5 }, _vertices[i * 4 + j], _vertices[i * 4 + j + 3]);
                    }
                }
            }
            if (_showControlPoints) {
                for (auto const & vertex : _vertices)
                    DrawPoint(tex, { 0.2, 0.7, 0.4 }, vertex);
            }
            _texture.Update(tex);
        }
        return Common::CaseRenderResult {
            .Fixed     = true,
            .Image     = _texture,
            .ImageSize = c_Size,
        };
    }

    void CaseImageWarping::OnProcessInput(ImVec2 const & pos) {
        auto         window  = ImGui::GetCurrentWindow();
        bool         hovered = false;
        bool         anyHeld = false;
        ImVec2 const delta   = ImGui::GetIO().MouseDelta;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hovered, &anyHeld);
        if (_selectIdx == -1 && ! hovered) return;
        if (ImGui::IsMouseDown(_enableLeftDrag ? ImGuiMouseButton_Left : ImGuiMouseButton_Right)) {
            _recompute = true;
            // find closest point
            if (_selectIdx == -1) {
                float minDist = 200;
                for (int i = 0; i < 16; ++i) {
                    float dist = (pos.x - _vertices[i].x) * (pos.x - _vertices[i].x) + (pos.y - _vertices[i].y) * (pos.y - _vertices[i].y);
                    if (dist < minDist) {
                        minDist    = dist;
                        _selectIdx = i;
                    }
                }
            }
            if (_selectIdx != -1) {
                glm::ivec2 targetPosition = _vertices[_selectIdx] + glm::ivec2(int(delta.x), int(delta.y));
                glm::ivec2 diff = targetPosition - c_ControlPoints[_selectIdx];
                if (std::abs(diff.x) <= c_ControlPointBound.x &&
                    std::abs(diff.y) <= c_ControlPointBound.y) {
                    if (_vertices[_selectIdx].x > 0 && _vertices[_selectIdx].x < int(c_Size.first - 1))
                        _vertices[_selectIdx].x = std::min(std::max(targetPosition.x, 1), int(c_Size.first - 2));
                    if (_vertices[_selectIdx].y > 0 && _vertices[_selectIdx].y < int(c_Size.first - 1))
                        _vertices[_selectIdx].y = std::min(std::max(targetPosition.y, 1), int(c_Size.second - 2));
                }
            }
        } else {
            _selectIdx = -1;
        }
        if (! _enableLeftDrag && ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.x != 0.f)
            ImGui::SetScrollX(window, window->Scroll.x - delta.x);
        if (! _enableLeftDrag && ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.y != 0.f)
            ImGui::SetScrollY(window, window->Scroll.y - delta.y);
        if (_enableZoom && ! anyHeld && ImGui::IsItemHovered())
            Common::ImGuiHelper::ZoomTooltip(_texture, c_Size, pos);
    }
} // namespace VCX::Labs::Drawing2D
