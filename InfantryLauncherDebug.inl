#pragma once

#ifndef INFANTRY_LAUNCHER_DEBUG_IMPL
#include "InfantryLauncher.hpp"
#endif

inline int InfantryLauncher::DebugCommand(int argc, char **argv) {
  enum class DebugView : uint8_t { STATE, MOTOR, HEAT, SHOT, FULL };

  constexpr uint8_t VIEW_STATE = static_cast<uint8_t>(DebugView::STATE);
  constexpr uint8_t VIEW_MOTOR = static_cast<uint8_t>(DebugView::MOTOR);
  constexpr uint8_t VIEW_HEAT = static_cast<uint8_t>(DebugView::HEAT);
  constexpr uint8_t VIEW_SHOT = static_cast<uint8_t>(DebugView::SHOT);
  constexpr uint8_t VIEW_FULL = static_cast<uint8_t>(DebugView::FULL);

  constexpr debug_core::ViewMask MASK_STATE = debug_core::view_bit(VIEW_STATE);
  constexpr debug_core::ViewMask MASK_MOTOR = debug_core::view_bit(VIEW_MOTOR);
  constexpr debug_core::ViewMask MASK_HEAT = debug_core::view_bit(VIEW_HEAT);
  constexpr debug_core::ViewMask MASK_SHOT = debug_core::view_bit(VIEW_SHOT);

  static constexpr std::array<debug_core::ViewEntry<uint8_t>, 5> VIEW_TABLE{{
      {"state", VIEW_STATE},
      {"motor", VIEW_MOTOR},
      {"heat", VIEW_HEAT},
      {"shot", VIEW_SHOT},
      {"full", VIEW_FULL},
  }};

#define LAUNCHER_MOTOR_FIELDS(name, member, mask)                               \
  DEBUG_CORE_LIVE_U8(InfantryLauncher, name "_state", (mask),                    \
                     (self->member).state),                                       \
      DEBUG_CORE_LIVE_F32(InfantryLauncher, name "_velocity", (mask),           \
                          (self->member).velocity),                              \
      DEBUG_CORE_LIVE_F32(InfantryLauncher, name "_omega", (mask),              \
                          (self->member).omega),                                 \
      DEBUG_CORE_LIVE_F32(InfantryLauncher, name "_torque", (mask),             \
                          (self->member).torque)

  static const debug_core::LiveFieldDesc<InfantryLauncher> FIELDS[] = {
      DEBUG_CORE_LIVE_U8(InfantryLauncher, "launcher_event", MASK_STATE,
                         self->launcher_event_),
      DEBUG_CORE_LIVE_U8(InfantryLauncher, "launcher_state", MASK_STATE,
                         self->launcher_state_),
      DEBUG_CORE_LIVE_U8(InfantryLauncher, "trig_mode", MASK_STATE,
                         self->trig_mode_),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "dt", MASK_STATE, self->dt_),
      DEBUG_CORE_LIVE_BOOL(InfantryLauncher, "is_fire_cmd", MASK_STATE,
                           self->launcher_cmd_.isfire),
      DEBUG_CORE_LIVE_BOOL(InfantryLauncher, "allow_fire", MASK_STATE,
                           self->heat_limit_.allow_fire),
      DEBUG_CORE_LIVE_BOOL(InfantryLauncher, "long_press", MASK_STATE,
                           self->press_continue_),
      DEBUG_CORE_LIVE_BOOL(InfantryLauncher, "shoot_active", MASK_STATE,
                           self->shoot_active_),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "target_rpm", MASK_MOTOR,
                          self->target_rpm_),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "trig_angle", MASK_MOTOR,
                          self->trig_angle_),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "trig_target_angle", MASK_MOTOR,
                          self->target_trig_angle_),
      LAUNCHER_MOTOR_FIELDS("fric_0", param_fric_0_, MASK_MOTOR),
      LAUNCHER_MOTOR_FIELDS("fric_1", param_fric_1_, MASK_MOTOR),
      LAUNCHER_MOTOR_FIELDS("trig", param_trig_, MASK_MOTOR),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "heat_now", MASK_HEAT,
                          self->heat_limit_.current_heat),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "heat_limit", MASK_HEAT,
                          self->referee_data_.heat_limit),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "heat_cooling", MASK_HEAT,
                          self->referee_data_.heat_cooling),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "heat_single", MASK_HEAT,
                          self->heat_limit_.single_heat),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "trig_freq", MASK_HEAT,
                          self->trig_freq_),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "number", MASK_SHOT, self->number_),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "shoot_dt", MASK_SHOT,
                          self->shoot_dt_),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "jam_keep_time_s", MASK_SHOT,
                          self->jam_keep_time_s_),
      DEBUG_CORE_LIVE_BOOL(InfantryLauncher, "jam_reverse", MASK_SHOT,
                           self->is_reverse_),
  };

#undef LAUNCHER_MOTOR_FIELDS

  return debug_core::run_live_command(
      this, "launcher", "state|motor|heat|shot|full", VIEW_TABLE, FIELDS,
      sizeof(FIELDS) / sizeof(FIELDS[0]), argc, argv, VIEW_FULL);
}
