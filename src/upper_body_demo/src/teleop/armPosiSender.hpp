// File: armPosiSender.hpp
#pragma once

#include "../IController.hpp"
#include "../CtrlContext.hpp"
#include "../axis.hpp"
#include "../power.hpp"
#include <array>

/// Implements your position‐control “wave hand” sequence for one joint
/// Accel is constant, by changing the time of accel and decel, all joints move and stop at the same time
///
class armPosiSender : public IController {
public:
  armPosiSender(axis_data&      axis,
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
  int    _print_cnt;
  float  _max_rot_accl; // rad/s/ms
  float  _max_rot_vel; // rad/s
  float  _max_lin_accl; // mm/s/ms
  float  _max_lin_vel; // mm/s

  // helpers
  static bool      in_enable_list(int id);
  bool             is_rot(int id);
  
  static int32_t   ang2enc_vel(double ang);
  static int32_t   len2enc_vel(double len);

  static float     enc2ang(int enc, int zero_posi);
  static float     enc2len(int enc, int zero_posi);

  static float     enc2ang_vel(int enc);
  static float     enc2len_vel(int enc);
  int              get_joint();
};
