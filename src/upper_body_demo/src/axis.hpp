#pragma once

#include <ecat/types.hpp>
#include <map>

struct axis_data
{
  std::uint16_t axis_id;
  std::uint16_t slave_pos;
  std::uint16_t joint_id;
  std::string joint_name;
  // bool tmp = false;


  // process data address
  volatile std::uint16_t *control_word;
  volatile std::uint16_t *error_code;

  volatile std::int32_t *target_position;
  volatile std::int32_t *velocity_offset;
  volatile std::int16_t *torque_offset;
  volatile std::int32_t *target_velocity;
  volatile std::int16_t *target_torque;
  volatile std::int8_t *mode_of_operation;

  // volatile const std::uint16_t *error_code;
  volatile const std::uint16_t *status_word;
  volatile const std::int32_t *position_actual_value;
  volatile const std::int32_t *velocity_actual_value;
  volatile const std::int16_t *torque_actual_value;
  volatile const std::int8_t *mode_of_operation_display;

  // volatile std::uint32_t *pv;
  // volatile std::uint32_t *pa;
  // volatile std::uint32_t *pd;

  std::uint16_t old_status_word;
  std::uint16_t old_control_word;
  int mode;
};
