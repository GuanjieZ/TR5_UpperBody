#pragma once

#include "axis.hpp"

class power {
public:
  power(axis_data& axis)
      : _axis(axis)
  {}

  // in
  bool enable = false;

  // out
  bool status;
  bool valid;
  bool error;
  int errorid;

  void on_cycle();

private:
  axis_data& _axis;
};
