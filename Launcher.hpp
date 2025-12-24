#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - motor_fric_front_left: '@&motor_fric_front_left'
  - motor_fric_front_right: '@&motor_fric_front_right'
  - motor_fric_back_left: '@&motor_fric_back_left'
  - motor_fric_back_right: '@&motor_fric_back_right'
  - motor_trig: '@&motor_trig'
  - cmd: '@&cmd'
  - task_stack_depth: 2048
  - launcher_param:
      fric1_setpoint_speed: 500.0
      fric2_setpoint_speed: 500.0
      default_bullet_speed: 0.0
      fric_radius: 6.0
      trig_gear_ratio: 19.0
      num_trig_tooth: 6
      trig_freq_: 0.0
  - pid_trig_angle:
      k: 1.0
      p: 4000.0
      i: 3000.0
      d: 0.0
      i_limit: 0.0
      out_limit: 2000.0
      cycle: false
  - pid_trig_speed:
      k: 1.0
      p: 0.0012
      i: 0.001
      d: 0.0
      i_limit: 1.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_0:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_1:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_2:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_3:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
template_args:
  - LauncherType: HeroLauncher
#  - FRIC_NUM: 4
  - TRIG_NUM: 1
required_hardware:
  - dr16
  - can
depends:
  - qdu-future/CMD
  - qdu-future/RMMotor
=== END MANIFEST === */
// clang-format on

#include <sys/_intsup.h>
#include <array>
#include <cstdint>

#include "CMD.hpp"
#include "HeroLauncher.hpp"
#include "InfantryLauncher.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "libxr_def.hpp"
#include "pid.hpp"

enum class LauncherEvent : uint8_t {
  SET_MODE_SAFE,
  SET_MODE_READY,
};

template <template<int, int> class LauncherType, int FRIC_NUM, int TRIG_NUM>
class Launcher : public LibXR::Application {
 public:
  struct LauncherParam {
    /*一级摩擦轮转速*/
    float fric1_setpoint_speed;
    /*二级摩擦轮转速*/
    float fric2_setpoint_speed;
    /*默认弹速*/
    float default_bullet_speed;
    /*摩擦轮半径*/
    float fric_radius;
    /*拨弹盘电机减速比*/
    float trig_gear_ratio;
    /*拨齿数目*/
    float num_trig_tooth;
    /*弹频*/
    float trig_freq_;
  };

  Launcher(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
           RMMotor* motor_fric_front_left, RMMotor* motor_fric_front_right,
           RMMotor* motor_fric_back_left, RMMotor* motor_fric_back_right,
           RMMotor* motor_trig, CMD* cmd, uint32_t task_stack_depth,
           LauncherParam launcher_param,
           LibXR::PID<float>::Param pid_trig_angle,
           LibXR::PID<float>::Param pid_trig_speed,
           LibXR::PID<float>::Param pid_fric_speed_0,
           LibXR::PID<float>::Param pid_fric_speed_1,
           LibXR::PID<float>::Param pid_fric_speed_2,
           LibXR::PID<float>::Param pid_fric_speed_3)
      : launcher_(hw, app,
                  std::array<RMMotor*, FRIC_NUM>{motor_fric_front_left, motor_fric_front_right,
                                                 motor_fric_back_left, motor_fric_back_right},
                  std::array<RMMotor*, TRIG_NUM>{motor_trig},
                  task_stack_depth, pid_trig_angle, pid_trig_speed,
                  pid_fric_speed_0, pid_fric_speed_1, pid_fric_speed_2, pid_fric_speed_3,
                  ::LauncherParam{
                      launcher_param.fric1_setpoint_speed, launcher_param.fric2_setpoint_speed,
                      launcher_param.default_bullet_speed, launcher_param.fric_radius,
                      launcher_param.trig_gear_ratio, launcher_param.num_trig_tooth,
                      launcher_param.trig_freq_}, cmd) {
    UNUSED(hw);
    UNUSED(app);
    UNUSED(cmd);
   // UNUSED(pid_fric_speed_2);
   // UNUSED(pid_fric_speed_3);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Launcher* launcher, uint32_t event_id) {
          UNUSED(in_isr);
          launcher->EventHandler(event_id);
        },
        this);

    launcher_event_.Register(static_cast<uint32_t>(LauncherEvent::SET_MODE_SAFE), callback);
    launcher_event_.Register(static_cast<uint32_t>(LauncherEvent::SET_MODE_READY), callback);
  }

  /**
   * @brief 获取发射器的事件处理器
   * @details 通过此事件处理器可以向发射器发送事件消息，控制发射器的行为模式
   * @return LibXR::Event& 事件处理器的引用
   */
  LibXR::Event& GetEvent() { return launcher_event_; }

  /**
   * @brief 事件处理器，根据传入的事件ID执行相应操作
   * @param event_id 触发的事件ID
   */
  void EventHandler(uint32_t event_id) { launcher_.SetMode(event_id); }

  void OnMonitor() override {}

 private:
  LauncherType<FRIC_NUM, TRIG_NUM> launcher_;
  LibXR::Event launcher_event_;
};
