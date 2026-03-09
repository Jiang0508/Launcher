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
  - task_stack_depth: 4096
  - pid_trig_angle:
      k: 1.0
      p: 4000.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 4000.0
      cycle: false
  - pid_trig_speed:
      k: 1.0
      p: 0.0012
      i: 0.0005
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
  - launcher_param:
      fric1_setpoint_speed: 4950.0
      fric2_setpoint_speed: 3820.0
      trig_gear_ratio: 19.2032
      num_trig_tooth: 6
      trig_freq_: 0.0
  - cmd: '@&cmd'
  - thread_priority: LibXR::Thread::Priority::HIGH
template_args:
  - LauncherType: HeroLauncher
required_hardware:
  - dr16
  - can
depends:
  - qdu-future/CMD
  - qdu-future/RMMotor
=== END MANIFEST === */
// clang-format on

#include <cstdint>

#include "CMD.hpp"
#include "HeroLauncher.hpp"
#include "InfantryLauncher.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "event.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"

#ifdef DEBUG
#include "DebugCore.hpp"
#include "ramfs.hpp"
#endif

template <class LauncherType>
class Launcher : public LibXR::Application {
 public:
  using LauncherEvent = typename LauncherType::LauncherEvent;

  struct LauncherParam {
    float fric1_setpoint_speed;
    float fric2_setpoint_speed;
    float trig_gear_ratio;
    uint8_t num_trig_tooth;
    float trig_freq_;
  };

  Launcher(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
           RMMotor* motor_fric_front_left, RMMotor* motor_fric_front_right,
           RMMotor* motor_fric_back_left, RMMotor* motor_fric_back_right,
           RMMotor* motor_trig, uint32_t task_stack_depth,
           LibXR::PID<float>::Param pid_trig_angle,
           LibXR::PID<float>::Param pid_trig_speed,
           LibXR::PID<float>::Param pid_fric_speed_0,
           LibXR::PID<float>::Param pid_fric_speed_1,
           LibXR::PID<float>::Param pid_fric_speed_2,
           LibXR::PID<float>::Param pid_fric_speed_3,
           LauncherParam launcher_param, CMD* cmd)
      : launcher_(hw, app, motor_fric_front_left, motor_fric_front_right,
                  motor_fric_back_left, motor_fric_back_right, motor_trig,
                  task_stack_depth, pid_trig_angle, pid_trig_speed,
                  pid_fric_speed_0, pid_fric_speed_1, pid_fric_speed_2,
                  pid_fric_speed_3,
                  typename LauncherType::LauncherParam{
                      launcher_param.fric1_setpoint_speed,
                      launcher_param.fric2_setpoint_speed,
                      launcher_param.trig_gear_ratio,
                      launcher_param.num_trig_tooth, launcher_param.trig_freq_},
                  cmd)
#ifdef DEBUG
        ,
        cmd_file_(LibXR::RamFS::CreateFile(
            "launcher",
            debug_core::command_thunk<LauncherType,
                                      &LauncherType::DebugCommand>,
            &launcher_))
#endif
  {
    UNUSED(app);

#ifdef DEBUG
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
#endif

    thread_.Create(this, ThreadFunc, "LauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::HIGH);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Launcher* self, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          self->mutex_.Lock();
          self->launcher_.LostCtrl();
          self->mutex_.Unlock();
        },
        this);

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Launcher* self, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          self->mutex_.Lock();
          self->launcher_.SetMode(
              static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX));
          self->mutex_.Unlock();
        },
        this);

    cmd->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
    cmd->GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);

    auto event_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Launcher* self, uint32_t event_id) {
          UNUSED(in_isr);
          self->mutex_.Lock();
          self->launcher_.SetMode(event_id);
          self->mutex_.Unlock();
        },
        this);
    launcher_event_.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX),
        event_callback);
    launcher_event_.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_SAFE),
        event_callback);
    launcher_event_.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_READY),
        event_callback);
  }

  LibXR::Event& GetEvent() { return launcher_event_; }

  void OnMonitor() override { launcher_.OnMonitor(); }

 private:
  LauncherType launcher_;
  LibXR::Event launcher_event_;
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

#ifdef DEBUG
  LibXR::RamFS::File cmd_file_;
#endif

  static void ThreadFunc(Launcher* self) {
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> cmd_sub("launcher_cmd");
    cmd_sub.StartWaiting();
    auto last_time = LibXR::Timebase::GetMilliseconds();

    while (true) {
      if (cmd_sub.Available()) {
        self->launcher_.launcher_cmd_ = cmd_sub.GetData();
        cmd_sub.StartWaiting();
      }

      self->mutex_.Lock();
      self->launcher_.Update();
      self->launcher_.Solve();
      self->mutex_.Unlock();
      self->launcher_.Control();

      LibXR::Thread::SleepUntil(last_time, 2);
    }
  }
};
