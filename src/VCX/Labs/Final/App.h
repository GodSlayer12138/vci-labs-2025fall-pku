#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/Final/CaseFixed.h"
#include "Labs/Final/CaseResizable.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::Final {
    class App : public Engine::IApp {
    private:
        Common::UI    _ui;

        CaseFixed     _caseFixed;
        CaseResizable _caseResizable;

        std::size_t   _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseFixed, _caseResizable };

    public:
        App();

        void OnFrame() override;
    };
}
