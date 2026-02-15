#pragma once

#ifndef INFANTRY_LAUNCHER_DEBUG_IMPL
#include "InfantryLauncher.hpp"
#endif

inline int InfantryLauncher::DebugCommand(int argc, char **argv) {
  enum class DebugView : uint8_t { STATE, MOTOR, HEAT, SHOT, FULL };

  constexpr uint8_t view_state = static_cast<uint8_t>(DebugView::STATE);
  constexpr uint8_t view_motor = static_cast<uint8_t>(DebugView::MOTOR);
  constexpr uint8_t view_heat = static_cast<uint8_t>(DebugView::HEAT);
  constexpr uint8_t view_shot = static_cast<uint8_t>(DebugView::SHOT);
  constexpr uint8_t view_full = static_cast<uint8_t>(DebugView::FULL);

  constexpr debug_core::ViewMask mask_state = debug_core::view_bit(view_state);
  constexpr debug_core::ViewMask mask_motor = debug_core::view_bit(view_motor);
  constexpr debug_core::ViewMask mask_heat = debug_core::view_bit(view_heat);
  constexpr debug_core::ViewMask mask_shot = debug_core::view_bit(view_shot);

  static constexpr std::array<debug_core::ViewEntry<uint8_t>, 5> view_table{{
      {"state", view_state},
      {"motor", view_motor},
      {"heat", view_heat},
      {"shot", view_shot},
      {"full", view_full},
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

  static const debug_core::LiveFieldDesc<InfantryLauncher> fields[] = {
      DEBUG_CORE_LIVE_U8(InfantryLauncher, "launcher_event", mask_state,
                         self->launcher_event_),
      DEBUG_CORE_LIVE_U8(InfantryLauncher, "launcher_state", mask_state,
                         self->launcher_state_),
      DEBUG_CORE_LIVE_U8(InfantryLauncher, "trig_mode", mask_state,
                         self->trig_mode_),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "dt", mask_state, self->dt_),
      DEBUG_CORE_LIVE_BOOL(InfantryLauncher, "is_fire_cmd", mask_state,
                           self->launcher_cmd_.isfire),
      DEBUG_CORE_LIVE_BOOL(InfantryLauncher, "allow_fire", mask_state,
                           self->heat_limit_.allow_fire),
      DEBUG_CORE_LIVE_BOOL(InfantryLauncher, "long_press", mask_state,
                           self->press_continue_),
      DEBUG_CORE_LIVE_BOOL(InfantryLauncher, "shoot_active", mask_state,
                           self->shoot_active_),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "target_rpm", mask_motor,
                          self->target_rpm_),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "trig_angle", mask_motor,
                          self->trig_angle_),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "trig_target_angle", mask_motor,
                          self->target_trig_angle_),
      LAUNCHER_MOTOR_FIELDS("fric_0", param_fric_0_, mask_motor),
      LAUNCHER_MOTOR_FIELDS("fric_1", param_fric_1_, mask_motor),
      LAUNCHER_MOTOR_FIELDS("trig", param_trig_, mask_motor),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "heat_now", mask_heat,
                          self->heat_limit_.current_heat),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "heat_limit", mask_heat,
                          self->referee_data_.heat_limit),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "heat_cooling", mask_heat,
                          self->referee_data_.heat_cooling),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "heat_single", mask_heat,
                          self->heat_limit_.single_heat),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "trig_freq", mask_heat,
                          self->trig_freq_),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "number", mask_shot, self->number_),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "shoot_dt", mask_shot,
                          self->shoot_dt_),
      DEBUG_CORE_LIVE_F32(InfantryLauncher, "jam_keep_time_s", mask_shot,
                          self->jam_keep_time_s_),
      DEBUG_CORE_LIVE_BOOL(InfantryLauncher, "jam_reverse", mask_shot,
                           self->is_reverse_),
  };

#undef LAUNCHER_MOTOR_FIELDS

  auto lock_self = +[](InfantryLauncher *self) { self->mutex_.Lock(); };
  auto unlock_self = +[](InfantryLauncher *self) { self->mutex_.Unlock(); };

  return debug_core::run_live_command(
      this, "launcher", "state|motor|heat|shot|full", view_table, fields,
      sizeof(fields) / sizeof(fields[0]), argc, argv, view_full, lock_self,
      unlock_self);
}
