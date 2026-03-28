#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_name: Launcher
module_description: Template launcher module supporting InfantryLauncher and HeroLauncher types
constructor_args:
  - task_stack_depth: 1536
  - launcher_param:
      fric_setpoint_speed: [6500.0, 0.0]
      trig_gear_ratio: 36.0
      num_trig_tooth: 10
      expect_trig_freq_: 16.0
      pid_trig_angle_:
        k: 1.0
        p: 40.0
        i: 0.1
        d: 0.0
        i_limit: 0.0
        out_limit: 0.0
        cycle: false
      pid_trig_speed_:
        k: 1.0
        p: 0.15
        i: 0.0
        d: 0.0
        i_limit: 0.0
        out_limit: 0.0
        cycle: false
      pid_fric_0:
        k: 0.8
        p: 0.0003
        i: 0.0
        d: 0.0
        i_limit: 0.0
        out_limit: 0.6
        cycle: false
      pid_fric_1:
        k: 0.8
        p: 0.0003
        i: 0.0
        d: 0.0
        i_limit: 0.0
        out_limit: 0.6
        cycle: false
      trig_motor_: '@&motor_trig'
      fric_motor_:
        - '@&motor_fric_0'
        - '@&motor_fric_1'
  - cmd: '@&cmd'
  - thread_priority: LibXR::Thread::Priority::HIGH
template_args:
  - LauncherType: InfantryLauncher
required_hardware:
  - dr16
  - can
depends:
  - qdu-future/CMD
  - qdu-future/RMMotor
=== END MANIFEST === */
// clang-format on

#include <array>
#include <cstdint>
#include <type_traits>

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
#include "Referee.hpp"
#ifdef DEBUG
#include "DebugCore.hpp"
#include "ramfs.hpp"
#endif

template <class LauncherType>
class Launcher : public LibXR::Application {
 public:
  using LauncherEvent = typename LauncherType::LauncherEvent;
  static constexpr int FRIC_NUM = LauncherType::FRIC_NUM;
  using LauncherParam = typename LauncherType::LauncherParam;

  Launcher(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
      uint32_t task_stack_depth, LauncherParam launcher_param, CMD* cmd,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH)
      : launcher_(hw, app, task_stack_depth, launcher_param, cmd)
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
                   thread_priority);

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
    LibXR::Topic::ASyncSubscriber<Referee::LauncherPack> launcher_ref(
        "launcher_ref");
    cmd_sub.StartWaiting();
    launcher_ref.StartWaiting();
    self->last_wakeup_time_ = LibXR::Timebase::GetMilliseconds();
    self->last_online_time_ = LibXR::Timebase::GetMicroseconds();

    while (true) {
      LibXR::Thread::Sleep(2);

      auto now = LibXR::Timebase::GetMicroseconds();
      self->launcher_.SetControlDt((now - self->last_online_time_).ToSecondf());
      self->last_online_time_ = now;

      if (cmd_sub.Available()) {
        self->launcher_.launcher_cmd_ = cmd_sub.GetData();
        cmd_sub.StartWaiting();
      }
      if(launcher_ref.Available()) {
        self->launcher_.ref_data_.heat_cooling = launcher_ref.GetData().rs.shooter_cooling_value;
        self->launcher_.ref_data_.heat_limit= launcher_ref.GetData().rs.shooter_heat_limit;
        launcher_ref.StartWaiting();
      }
      self->mutex_.Lock();
      self->launcher_.Update();
      self->launcher_.Solve();
      self->mutex_.Unlock();
      self->launcher_.Control();
    }
  }

  LibXR::MillisecondTimestamp last_wakeup_time_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;
};
