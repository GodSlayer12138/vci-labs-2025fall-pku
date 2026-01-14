#pragma once

#include "Engine/GL/Frame.hpp"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Final/Mixbox.h"

namespace VCX::Labs::Final {

    class CaseMixbox : public Common::ICase {
    public:
        CaseMixbox();

        virtual std::string_view const GetName() override { return "Mixbox Color Mixing"; }
        
        virtual void OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void OnProcessInput(ImVec2 const & pos) override;

    private:
        void ClearCanvas();
        void DrawBrushStroke(int x, int y, float dirX = 1.0f, float dirY = 0.0f);
        glm::vec3 GetCurrentBrushColor() const;

        Engine::GL::UniqueRenderFrame _frame;
        Common::ImageRGB              _canvas;
        Engine::GL::UniqueTexture2D   _texture;

        // Color mixing parameters (0-255 range)
        int _color1[3] = {0, 33, 133};    // Cobalt Blue
        int _color2[3] = {254, 211, 0};    // Hansa Yellow
        float _mixRatio = 0.5f;

        // Brush parameters
        int   _brushSize       = 15;

        // Canvas parameters
        int   _canvasWidth     = 1600;
        int   _canvasHeight    = 1200;

        // Drawing state
        bool  _isDrawing       = false;
        ImVec2 _lastDrawPos    = ImVec2(-1, -1);
        bool  _canvasDirty     = true;  // Track if canvas needs texture update
        
        // Cached brush color (avoid recomputing every pixel)
        glm::vec3 _cachedBrushColor = glm::vec3(0.0f);
        unsigned char _cachedBrushR = 0, _cachedBrushG = 0, _cachedBrushB = 0;
    };

} // namespace VCX::Labs::Final
