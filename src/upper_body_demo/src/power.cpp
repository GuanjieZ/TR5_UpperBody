#include <iostream>
#include <spdlog/spdlog.h>
#include <optional>
#include "power.hpp"
#include "bit_pattern.hpp"

enum class cia402_state
{
  none = -1,
  not_ready_to_switch_on = 0,
  switch_on_disabled,
  ready_to_switch_on,
  switched_on,
  operation_enabled,
  quick_stop_active,
  fault_reaction_active,
  fault,
};

const char *to_string(cia402_state state)
{
  switch (state)
  {
  case cia402_state::none:
    return "none";
  case cia402_state::not_ready_to_switch_on:
    return "not ready to switch on";
  case cia402_state::switch_on_disabled:
    return "switch on disabled";
  case cia402_state::ready_to_switch_on:
    return "ready to switch on";
  case cia402_state::switched_on:
    return "switched on";
  case cia402_state::operation_enabled:
    return "operation enabled";
  case cia402_state::quick_stop_active:
    return "quick stop active";
  case cia402_state::fault_reaction_active:
    return "fault reaction active";
  case cia402_state::fault:
    return "fault";
  default:
    return "invalid";
  }
}

using u16_bit_pattern = basic_bit_pattern<std::uint16_t>;

static constexpr std::array<std::pair<u16_bit_pattern, cia402_state>, 8>
    state_patterns = {{
        {{0x004f, 0x0040}, cia402_state::switch_on_disabled},
        {{0x006f, 0x0021}, cia402_state::ready_to_switch_on},
        {{0x006f, 0x0023}, cia402_state::switched_on},
        {{0x006f, 0x0027}, cia402_state::operation_enabled},
        {{0x006f, 0x0007}, cia402_state::quick_stop_active},
        {{0x004f, 0x000f}, cia402_state::fault_reaction_active},
        {{0x004f, 0x0008}, cia402_state::fault},
        {{0x0000, 0x0000}, cia402_state::not_ready_to_switch_on},
    }};

constexpr cia402_state to_cia402_state(std::uint16_t status_word)
{
  for (auto [pattern, state] : state_patterns)
    if (pattern == status_word)
      return state;
  return cia402_state::fault;
}

static constexpr std::array<std::pair<cia402_state, u16_bit_pattern>, 8>
    enable_transition_commands = {{
        {cia402_state::not_ready_to_switch_on, {0x0081 | 0x0006, 0x0006}},
        {cia402_state::switch_on_disabled, {0x0081 | 0x0006, 0x0006}},
        {cia402_state::ready_to_switch_on, {0x0088 | 0x0007, 0x0007}},
        {cia402_state::switched_on, {0x0080 | 0x000F, 0x000F}},
        {cia402_state::operation_enabled, {0, 0}},
        {cia402_state::quick_stop_active, {0, 0}},
        {cia402_state::fault_reaction_active, {0, 0}},
        {cia402_state::fault, {0x0080, 0x0080}},
    }};

static constexpr std::array<std::pair<cia402_state, u16_bit_pattern>, 5>
    disable_transition_commands = {{
        {cia402_state::not_ready_to_switch_on, {0x0081 | 0x0006, 0x0006}},
        {cia402_state::switch_on_disabled, {0x0081 | 0x0006, 0x0006}},
        {cia402_state::ready_to_switch_on, {0x0088 | 0x0007, 0x0007}},
        {cia402_state::switched_on, {0x0080 | 0x000F, 0x0007}},
        {cia402_state::operation_enabled, {0x0080 | 0x000F, 0x0007}},
    }};

auto get_enable_command(cia402_state state) -> std::optional<u16_bit_pattern>
{
  for (auto [s, c] : enable_transition_commands)
  {
    if (s == state)
      return c;
  }
  return std::nullopt;
}

auto get_disable_command(cia402_state state) -> std::optional<u16_bit_pattern>
{
  for (auto [s, c] : disable_transition_commands)
  {
    if (s == state)
      return c;
  }
  return std::nullopt;
}

enum mode_of_operation_type : std::uint8_t
{
  op_mode_no = 0,
  op_mode_pp = 1,
  op_mode_vl = 2,
  op_mode_pv = 3,
  op_mode_hm = 6,
  op_mode_ip = 7,
  op_mode_csp = 8,
  op_mode_csv = 9,
  op_mode_cst = 10,
};

void power::on_cycle()
{
  // printf(".......... power_On_cycle axis: %d\n", axis->axis_id);
  if (!_axis.control_word) {
        printf("[POWER ERROR] axis->control_word is null (axis_id = %d)\n", _axis.axis_id);
        return;
    }

  if (!_axis.status_word) {
        printf("[POWER ERROR] axis->status_word is null (axis_id = %d)\n", _axis.axis_id);
        return;
    }

  //将状态字转换成状态
  auto state = to_cia402_state(*_axis.status_word);

  std::optional<u16_bit_pattern> command;
  //针对不同的enable状态，获取对应的比特位指令
  if (enable)
  {
    command = get_enable_command(state);
  }
  else
  {
    command = get_disable_command(state);
  }

  if (!command)
  {
    error = true;
    errorid = -1;
  }
  else
  {
    // __RT(printf("command: 0x%04x | %04o \n", command, command));
    //用上面获取的比特位指令设置对应的控制字
    *_axis.control_word << *command;
    // __RT(printf("control_word: 0x%04x | %04o \n", *axis->control_word, *axis->control_word));
    error = false;
    errorid = 0;
    
    //在switched_on的状态下设置运行模式
    if (state == cia402_state::switched_on)
    {
      if (_axis.mode_of_operation)
      {
        *_axis.mode_of_operation = mode_of_operation_type(_axis.mode);
      }
    }
  }

  valid = true;

  auto old_state = to_cia402_state(_axis.old_status_word);

  if (old_state != state)
  {
    spdlog::debug("Axis {} changed from '{}' to '{}', old_ctrl=0x{:X}, new_ctrl=0x{:X}, error_code: 0x{:X}, old_status0x{:X}, new_status0x{:X}",
      _axis.axis_id,
      to_string(old_state),
      to_string(state),
      _axis.old_control_word,
      *_axis.control_word,
      *_axis.error_code,
      _axis.old_status_word,
      *_axis.status_word
      );

    _axis.old_status_word = *_axis.status_word;
    _axis.old_control_word = *_axis.control_word;
  }
  
  //只有当状态为operation_enabled的时候才将status设置为true
  status = state == cia402_state::operation_enabled;
}
