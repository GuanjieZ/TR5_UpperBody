#pragma once

#include "../axis.hpp"
#include "../power.hpp"
#include "../IController.hpp"
#include "WaveHandDemo.hpp"
#include <memory>
#include <spdlog/spdlog.h>

class Joint {
public:
    Joint(axis_data& axis,
          CtrlContext& ctx)
        : _axis(axis)
        , _power(axis)
        , _ctrl(std::make_unique<WaveHandDemo>(_axis, _power, ctx))
    {}

    void operator()() {
        _power.on_cycle();
        _ctrl->on_cycle();
    }

    void disablePower() {
        _power.enable = false;
        _power.on_cycle();
    }

private:
    axis_data& _axis;
    power _power;
    std::unique_ptr<IController> _ctrl;
};