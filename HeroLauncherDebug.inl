#pragma once

#ifndef HERO_LAUNCHER_DEBUG_IMPL
#include "HeroLauncher.hpp"
#endif

inline int HeroLauncher::DebugCommand(int argc, char** argv) {
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

#define HERO_MOTOR_FIELDS(name, member, mask)                                \
  DEBUG_CORE_LIVE_U8(HeroLauncher, name "_state", (mask),                    \
                     (self->member).state),                                   \
      DEBUG_CORE_LIVE_F32(HeroLauncher, name "_velocity", (mask),            \
                          (self->member).velocity),                           \
      DEBUG_CORE_LIVE_F32(HeroLauncher, name "_omega", (mask),               \
                          (self->member).omega),                              \
      DEBUG_CORE_LIVE_F32(HeroLauncher, name "_torque", (mask),              \
                          (self->member).torque)

  static const debug_core::LiveFieldDesc<HeroLauncher> FIELDS[] = {
      DEBUG_CORE_LIVE_CUSTOM(
          HeroLauncher, "launcher_event", MASK_STATE,
          +[](const char* field_name, const HeroLauncher* self) {
            const char* text = "UNKNOWN";
            switch (self->launcher_event_) {
              case LauncherEvent::SET_FRICMODE_RELAX:
                text = "RELAX";
                break;
              case LauncherEvent::SET_FRICMODE_SAFE:
                text = "SAFE";
                break;
              case LauncherEvent::SET_FRICMODE_READY:
                text = "READY";
                break;
            }
            LibXR::STDIO::Printf("  %s=%s\r\n", field_name, text);
          }),
      DEBUG_CORE_LIVE_CUSTOM(
          HeroLauncher, "trig_mode", MASK_STATE,
          +[](const char* field_name, const HeroLauncher* self) {
            const char* text = "UNKNOWN";
            switch (self->trig_mode_) {
              case TrigMode::RELAX:
                text = "RELAX";
                break;
              case TrigMode::SAFE:
                text = "SAFE";
                break;
              case TrigMode::SINGLE:
                text = "SINGLE";
                break;
              case TrigMode::CONTINUE:
                text = "CONTINUE";
                break;
            }
            LibXR::STDIO::Printf("  %s=%s\r\n", field_name, text);
          }),
      DEBUG_CORE_LIVE_F32(HeroLauncher, "dt", MASK_STATE, self->dt_),
      DEBUG_CORE_LIVE_BOOL(HeroLauncher, "is_fire_cmd", MASK_STATE,
                           self->launcher_cmd_.isfire),
      DEBUG_CORE_LIVE_BOOL(HeroLauncher, "first_loading", MASK_STATE,
                           self->first_loading_),
      DEBUG_CORE_LIVE_BOOL(HeroLauncher, "fire_flag", MASK_STATE,
                           self->fire_flag_),
      DEBUG_CORE_LIVE_BOOL(HeroLauncher, "enable_fire", MASK_STATE,
                           self->enable_fire_),
      DEBUG_CORE_LIVE_BOOL(HeroLauncher, "mark_launch", MASK_STATE,
                           self->mark_launch_),
      DEBUG_CORE_LIVE_BOOL(HeroLauncher, "press_continue", MASK_STATE,
                           self->press_continue_),
      DEBUG_CORE_LIVE_F32(HeroLauncher, "trig_angle", MASK_MOTOR,
                          self->trig_angle_),
      DEBUG_CORE_LIVE_F32(HeroLauncher, "trig_setpoint_angle", MASK_MOTOR,
                          self->trig_setpoint_angle_),
      DEBUG_CORE_LIVE_F32(HeroLauncher, "trig_output", MASK_MOTOR,
                          self->trig_output_),
      HERO_MOTOR_FIELDS("fric_fl", param_motor_fric_front_left_, MASK_MOTOR),
      HERO_MOTOR_FIELDS("fric_fr", param_motor_fric_front_right_, MASK_MOTOR),
      HERO_MOTOR_FIELDS("fric_bl", param_motor_fric_back_left_, MASK_MOTOR),
      HERO_MOTOR_FIELDS("fric_br", param_motor_fric_back_right_, MASK_MOTOR),
      HERO_MOTOR_FIELDS("trig", param_trig_, MASK_MOTOR),
      DEBUG_CORE_LIVE_F32(HeroLauncher, "heat", MASK_HEAT,
                          self->heat_ctrl_.heat),
      DEBUG_CORE_LIVE_F32(HeroLauncher, "heat_limit", MASK_HEAT,
                          self->heat_ctrl_.heat_limit),
      DEBUG_CORE_LIVE_F32(HeroLauncher, "cooling_rate", MASK_HEAT,
                          self->heat_ctrl_.cooling_rate),
      DEBUG_CORE_LIVE_F32(HeroLauncher, "heat_increase", MASK_HEAT,
                          self->heat_ctrl_.heat_increase),
      DEBUG_CORE_LIVE_U8(HeroLauncher, "available_shot", MASK_HEAT,
                         self->heat_ctrl_.available_shot),
      DEBUG_CORE_LIVE_U8(HeroLauncher, "fired", MASK_SHOT, self->fired_),
      DEBUG_CORE_LIVE_F32(HeroLauncher, "real_launch_delay_ms", MASK_SHOT,
                          self->real_launch_delay_),
      DEBUG_CORE_LIVE_F32(HeroLauncher, "fric_target_0", MASK_SHOT,
                          self->fric_target_speed_[0]),
      DEBUG_CORE_LIVE_F32(HeroLauncher, "fric_target_1", MASK_SHOT,
                          self->fric_target_speed_[1]),
      DEBUG_CORE_LIVE_F32(HeroLauncher, "fric_target_2", MASK_SHOT,
                          self->fric_target_speed_[2]),
      DEBUG_CORE_LIVE_F32(HeroLauncher, "fric_target_3", MASK_SHOT,
                          self->fric_target_speed_[3]),
      DEBUG_CORE_LIVE_F32(HeroLauncher, "current_back_left", MASK_SHOT,
                          self->current_back_left_),
  };

#undef HERO_MOTOR_FIELDS

  return debug_core::run_live_command(
      this, "launcher", "state|motor|heat|shot|full", VIEW_TABLE, FIELDS,
      sizeof(FIELDS) / sizeof(FIELDS[0]), argc, argv, VIEW_FULL);
}
