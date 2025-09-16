#pragma once

struct IController {
    virtual ~IController() = default;

    virtual void on_cycle() = 0;
};