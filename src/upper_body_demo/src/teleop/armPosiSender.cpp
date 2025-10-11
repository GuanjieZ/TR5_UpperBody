// File: armPosiSender.cpp
#include "armPosiSender.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>  // for std::clamp
#include <cstdio>     // for printf
#include <cmath>      // for std::abs

#define MOTION_FLAG 0
constexpr double PI = 3.14159265358979323846;

armPosiSender::armPosiSender(axis_data& axis,
                           power& power_ctrl,
                           CtrlContext& ctx)
  : _axis(axis)
  , _power(power_ctrl)
  , _ctx(ctx)
//   , _joint_map{0,1,
//                4,3,2,6,7,8,5, 
//                11,10,9,13,14,15,12}
  , _joint_map{0,1,
               4,3,2,5,8,7,6, 
               11,10,9,13,15,14,12}
  , _initialized(false)
  , _init_pos(0)
  , _last_step(-1)
  , _print_cnt(0)
  , _max_rot_accl(1.0472*10/1000.0)
  , _max_rot_vel(1.0472/1)
  , _max_lin_accl(20.0*10/1000.0)
  , _max_lin_vel(20.0/1)

{}

void armPosiSender::on_cycle() { 
    
    bool is_test = true;
    _print_cnt++;
    _print_cnt = _print_cnt % 100000;
    // 1) Safety checks
    if (!*_axis.position_actual_value) return;

    // 2) Enable/disable per list
    // _power.enable = _ctx.enable_flags[_axis.joint_id];
    // if (_axis.joint_id != 8)_power.enable = _ctx.enable_flags[_axis.joint_id];
    // else _power.enable = false;

    if (_axis.joint_id > 8)_power.enable = _ctx.enable_flags[_axis.joint_id];
    else _power.enable = false;
    
    // if (_print_cnt % 500 == 0) {
    // spdlog::info("joint:{} | pos:{:07d} | vel:{:07d} | cmd_vel:{:6.4f} | tgt_vel:{:6.4f} | enable:{}",
    //         _axis.joint_name,
    //         *_axis.position_actual_value,
    //         *_axis.velocity_actual_value,
    //         _ctx.command_q_dot[_axis.joint_id],
    //         _ctx.target_q_dot[_axis.joint_id],
    //         _ctx.enable_flags[_axis.joint_id]
    //         );
    //     if (_axis.axis_id == 15){
    //         spdlog::info("    ");
    //     }
    // }

    if (is_rot(get_joint())) {
        // record joint state
        _ctx.q[get_joint()] = enc2ang(*_axis.position_actual_value, _ctx.joint_zero_posi[get_joint()]);
        _ctx.q_rot[get_joint()] = _ctx.q[get_joint()];

        // update command velocity
        float err = (_ctx.target_q[_axis.joint_id] - _ctx.q[_axis.joint_id]); 
        if (fabsf(err) < 0.0001) err = 0.0f;
        _ctx.q_err[_axis.joint_id] += err * 0.001;
        float new_qdot = _ctx.qdot_kp[_axis.joint_id]*4.0 * (_ctx.target_q[_axis.joint_id] - _ctx.q[_axis.joint_id]) + _ctx.qdot_ki[_axis.joint_id]*1.0 * _ctx.q_err[_axis.joint_id];
        _ctx.command_q_dot[_axis.joint_id]  += std::clamp(new_qdot - _ctx.command_q_dot[_axis.joint_id], -_max_rot_accl, _max_rot_accl);
        _ctx.command_q_dot[_axis.joint_id] = std::clamp(_ctx.command_q_dot[_axis.joint_id], -_max_rot_vel, _max_rot_vel);
        _ctx.command_q_dot[_axis.joint_id] = _ctx.command_q_dot[_axis.joint_id];
        *_axis.target_velocity = ang2enc_vel(_ctx.command_q_dot[_axis.joint_id]);

    } else {
        // record joint state
        _ctx.q[get_joint()] = enc2len(*_axis.position_actual_value, _ctx.joint_zero_posi[get_joint()]);

        // update command velocity
        float err = (_ctx.target_q[_axis.joint_id] - _ctx.q[_axis.joint_id]);
        if (fabsf(err) < 0.0001) err = 0.0f;
        _ctx.q_err[_axis.joint_id] += err * 0.001;
        float new_qdot = _ctx.qdot_kp[_axis.joint_id]*12.0 * (_ctx.target_q[_axis.joint_id] - _ctx.q[_axis.joint_id]) + _ctx.qdot_ki[_axis.joint_id]*3.0 * _ctx.q_err[_axis.joint_id];
        _ctx.command_q_dot[_axis.joint_id]  += std::clamp(new_qdot - _ctx.command_q_dot[_axis.joint_id], -_max_lin_accl, _max_lin_accl);
        _ctx.command_q_dot[_axis.joint_id] = std::clamp(_ctx.command_q_dot[_axis.joint_id], -_max_lin_vel, _max_lin_vel);
        _ctx.command_q_dot[_axis.joint_id] = _ctx.command_q_dot[_axis.joint_id];
        *_axis.target_velocity = len2enc_vel(_ctx.command_q_dot[_axis.joint_id]);
    }

    

    if (_print_cnt % 500 == 0) {
    spdlog::info("joint:{} | q:{:8.5f} | tgt_q:{:8.5f} | vel:{:07d} | cmd_vel:{:8.5f} | tgt_vel:{:8.5f} | enable:{}",
            _axis.joint_name,
            _ctx.q[_axis.joint_id],
            _ctx.target_q[_axis.joint_id],
            *_axis.velocity_actual_value,
            _ctx.command_q_dot[_axis.joint_id],
            _ctx.target_q_dot[_axis.joint_id],
            _ctx.enable_flags[_axis.joint_id]
            );
        if (_axis.axis_id == 15){
            spdlog::info("    ");
        }
    }

}

bool armPosiSender::is_rot(int id) {
    static constexpr int list[] = {7,8,14,15}; 
    for (int e : list) if (e == id) return false;
    return true;
}

int32_t armPosiSender::ang2enc_vel(double ang) {
    return int32_t(ang * 131071.0 / (2*PI));
}

int32_t armPosiSender::len2enc_vel(double len) {
    return int32_t(len / 2.0 * 131071.0);
}

float armPosiSender::enc2ang(int enc, int zero_posi) {
    return float((enc - zero_posi)/ 131071.0 * 2 * PI);
}

float armPosiSender::enc2len(int enc, int zero_posi) {
    return float((enc - zero_posi) / 131071.0 * 2.0);
}

float armPosiSender::enc2ang_vel(int enc) {
    return float(enc/ 131071.0 * 2 * PI);
}

float armPosiSender::enc2len_vel(int enc) {
    return float(enc / 131071.0 * 2.0);
}


int armPosiSender::get_joint(){
    return _joint_map[_axis.axis_id];
}
