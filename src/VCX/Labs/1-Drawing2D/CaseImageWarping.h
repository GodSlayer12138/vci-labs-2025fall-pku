#pragma once

#include "Engine/Async.hpp"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"

namespace VCX::Labs::Drawing2D {

    class CaseImageWarping : public Common::ICase {
    public:
        CaseImageWarping();

        virtual std::string_view const GetName() override { return "Image Warping"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        Engine::GL::UniqueTexture2D _texture;

        Common::ImageRGB _empty;
        Common::ImageRGB _input;

        Engine::Async<Common::ImageRGB> _task;

        bool _enableZoom        = true;
        bool _enableLeftDrag    = false;
        bool _showControlPoints = true;
        bool _showGrids         = true;
        bool _showImage         = true;
        bool _recompute         = true;

        int                        _selectIdx = -1;
        std::array<glm::ivec2, 16> _vertices;
    };
} // namespace VCX::Labs::Drawing2D
