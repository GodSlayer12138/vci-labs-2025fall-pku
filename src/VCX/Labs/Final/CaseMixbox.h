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

        // Color palette (16 colors: 13 presets + 3 custom)
        static constexpr int PALETTE_SIZE = 16;
        int _palette[PALETTE_SIZE][3] = {
            {254, 236, 0},   // Cadmium Yellow
            {252, 211, 0},   // Hansa Yellow
            {255, 105, 0},   // Cadmium Orange
            {255, 39, 2},    // Cadmium Red
            {128, 2, 46},    // Quinacridone Magenta
            {78, 0, 66},     // Cobalt Violet
            {25, 0, 89},     // Ultramarine Blue
            {0, 33, 133},    // Cobalt Blue
            {13, 27, 68},    // Phthalo Blue
            {0, 60, 50},     // Phthalo Green
            {7, 109, 22},    // Permanent Green
            {107, 148, 4},   // Sap Green
            {123, 72, 0},    // Burnt Sienna
            {255, 255, 255}, // Custom 1
            {255, 255, 255}, // Custom 2
            {255, 255, 255}  // Custom 3
        };
        int _selectedColorIndex = 0;  // Currently selected color

        // Color mixer parameters
        int _mixerColor1Index = 1;  // Hansa Yellow (252, 211, 0)
        int _mixerColor2Index = 7;  // Cobalt Blue (0, 33, 133)
        float _mixRatio = 0.5f;     // Mix ratio between color1 and color2

        // Brush parameters
        int   _brushSize       = 30;
        int   _brushOpacity    = 100;  // Brush opacity percentage (0-100%)

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
