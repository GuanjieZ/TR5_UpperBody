// File: WaveHandDemo.hpp
#pragma once

#include "../IController.hpp"
#include "../CtrlContext.hpp"
#include "../axis.hpp"
#include "../power.hpp"
#include <array>

/// Implements your position‐control “wave hand” sequence for one joint
/// Accel is constant, by changing the time of accel and decel, all joints move and stop at the same time
///
class WaveHandDemo : public IController {
public:
  WaveHandDemo(axis_data&      axis,
               power&          power_ctrl,
               CtrlContext& ctx);

  void on_cycle() override;

private:
  axis_data&           _axis;
  power&               _power;
  CtrlContext&      _ctx;
  std::array<int, num_joints> _joint_map;

  // per‐joint state
  bool   _initialized;
  int    _init_pos;
  int    _last_step;
  int    _end_pos;
  double    _target_vel;
  int    _print_cnt;
  int    _motion_cnt;
  double    _vel_max;
  double  _accel_max;
  int    _accel_cnt;
  int    _decel_cnt;
  int    _end_cnt;
  double _time_amp;
  int    _direct;

  // helpers
  static bool      in_enable_list(int id);
  void             init_validation();
  int              plan(int step, double dt); // based on distance, calculate accel_cnt and decel_cnt
  void             update_motion(int direct);
  static int32_t   deg_to_enc(double deg);
  int              get_joint();
};
