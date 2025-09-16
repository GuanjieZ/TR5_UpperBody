// File: WaveHandDemo.cpp
#include "WaveHandDemo.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>  // for std::clamp
#include <cstdio>     // for printf
#include <cmath>      // for std::abs
#define MOTION_FLAG 0
WaveHandDemo::WaveHandDemo(axis_data& axis,
                           power& power_ctrl,
                           CtrlContext& ctx)
  : _axis(axis)
  , _power(power_ctrl)
  , _ctx(ctx)
  , _joint_map{0,1,
               4,3,2,6,7,8,5, 
               11,10,9,13,14,15,12}
  , _initialized(false)
  , _init_pos(0)
  , _last_step(-1)
  , _end_pos(0)
  , _print_cnt(0)

  , _target_vel(0.0)
  , _motion_cnt(0)
  , _vel_max(21845.0)
  , _accel_max(63.69) // acceleration in cnt/s/ms
  , _accel_cnt(-1)
  , _decel_cnt(100000)
  , _end_cnt(0)
  , _time_amp(3.0)
  , _direct(0)
{}

void WaveHandDemo::on_cycle() {
    _print_cnt++;
    _print_cnt = _print_cnt % 100000;
    // 1) Safety checks
    if (!*_axis.position_actual_value) return;

    // 2) Enable/disable per list
    // _power.enable = in_enable_list(_joint_map[_axis.axis_id]);
    // if (get_joint() == 4) {
    //     _power.enable = true;
    // } else {
    //     _power.enable = false;
    // }
    _power.enable = false;

    if (_print_cnt % 100 == 0) {
    spdlog::info("joint:{} | pos:{} | vel:{} | cmd_vel:{:.6f} | ac_cnt:{} | dc_cnt:{} | step:{} test",
            get_joint(),
            // _axis.axis_id,
            *_axis.position_actual_value,
            *_axis.velocity_actual_value,
            _target_vel,
            _accel_cnt,
            _decel_cnt,
            _ctx.step_index
            );
        if (_axis.axis_id == 15){
            spdlog::info("    ");
        }
    }

    // // Record the init value
    // if (!_initialized) {
    // _init_pos    = *_axis.position_actual_value;
    // _initialized = true;
    // }

    // // wait for all encoder readings to be valid
    // if (_ctx.step_index < 0) {
    // *_axis.target_velocity = 0;
    // init_validation();
    // return;
    // }

    // //plan
    // if (_ctx.plan_flag[_joint_map[_axis.axis_id]] && _ctx.step_index >= 0 && in_enable_list(get_joint())) {
    // _direct = 0;
    // _direct = plan(_ctx.step_index, _ctx.time_window[_ctx.step_index]);
    // _ctx.plan_flag[_joint_map[_axis.axis_id]] = false;
    // }

    // // Enter quick stop mode when global move is false or encoder reading is zero
    // _ctx.move_allowed[_joint_map[_axis.axis_id]] = (*_axis.position_actual_value != 0);
    // if (!_ctx.all_move_allowed || *_axis.position_actual_value == 0) return;

    // // Skip if joint_done for this joint is set to true
    // if (_ctx.joint_done[_joint_map[_axis.axis_id]]) return;

    // // Update velocity
    // update_motion(_direct);
}

bool WaveHandDemo::in_enable_list(int id) {
    static constexpr int list[] = {0,1,
                                   2,3,4,5,6,
                                   9,10,11,12,13};
    for (int e : list) if (e == id) return true;
    return false;
}

void WaveHandDemo::init_validation() {
    int j = _joint_map[_axis.axis_id];
    if (!_init_pos || _init_pos == 131071 && j != 3) {
    _ctx.joint_done[j] = false;
    }
    else {
    _ctx.joint_done[j] = true;
    _init_pos = *_axis.position_actual_value;
    }
}

int WaveHandDemo::plan(int step, double dt) {
    dt = dt * _time_amp;
    _end_cnt = static_cast<int>(dt * 1000);
    int j     = _joint_map[_axis.axis_id];
    int start_pos = *_axis.position_actual_value;
    _end_pos  = _ctx.joint_zero_posi[j] + deg_to_enc(_ctx.target_joint_posi[step][j]);
    int distance = _end_pos - start_pos;
    int sign = (distance > 0) - (distance < 0);
    _end_cnt = static_cast<int>(dt*1000);

    // Calculate the delta function value
    double delta = dt * dt - 4.0 * std::abs(distance) / (_accel_max * 1000.0);
    
    // If delta < 0, time window is too small thus not valid.
    if (delta <= 0) {
        spdlog::info("negative delta, time window too small. joint:{}", get_joint());
    } else {
        _accel_cnt = static_cast<int>((dt - sqrt(delta)) / 2 * 1000);
        _decel_cnt = static_cast<int>((dt + sqrt(delta)) / 2 * 1000);
    }    

    return sign;
}

void WaveHandDemo::update_motion(int direct) {
    if (_ctx.end) {
        *_axis.target_velocity = 0;
        return;
    }

    _motion_cnt ++;

    if (_motion_cnt < _accel_cnt) {
        if (get_joint() == 3) {
            _target_vel += direct * _accel_max * 1;
        } else {
            _target_vel += direct * _accel_max;
        }
    } else if (_motion_cnt >= _end_cnt){
        _target_vel = 0.0;
        _accel_cnt = -1;
        _decel_cnt = 100000;
        _motion_cnt = 0;
        _ctx.joint_done[_joint_map[_axis.axis_id]] = true;
        spdlog::info("joint:{} | pos:{} | vel:{} | cmd_vel:{:.6f} | ac_cnt:{} | dc_cnt:{} | step:{} | m_cnt:{}",
        get_joint(),
        *_axis.position_actual_value,
        *_axis.velocity_actual_value,
        _target_vel,
        _accel_cnt,
        _decel_cnt,
        _ctx.step_index,
        _motion_cnt
        );
    } else if (_motion_cnt >= _decel_cnt) {
        if (get_joint() == 3) {
            _target_vel -= direct * _accel_max * 1;
        } else {
            _target_vel -= direct * _accel_max;
        }
    }

    _target_vel = std::clamp(_target_vel, -_vel_max, _vel_max);

    *_axis.target_velocity = static_cast<int>(_target_vel);
}

int32_t WaveHandDemo::deg_to_enc(double deg) {
    return int32_t(deg * 131071.0 / 360.0);
}

int WaveHandDemo::get_joint(){
    return _joint_map[_axis.axis_id];
}
