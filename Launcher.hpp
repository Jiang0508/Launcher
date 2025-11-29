#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - task_stack_depth: 2048
  - pid_param_trig_angle:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_param_trig_speed:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_param_fric:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - motor_fric_0: '@&motor_fric_0'
  - motor_fric_1: '@&motor_fric_1'
  - motor_trig: '@&motor_trig'
  - launcher_param:
      default_bullet_speed: 0.0
      fric_radius: 0.0
      trig_gear_ratio: 0.0
      num_trig_tooth: 0
      trig_freq_: 0.0
template_args:
required_hardware:
  - dr16
  - can
depends:
  - qdu-future/CMD
  - qdu-future/RMMotor
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>

#include "CMD.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "timer.hpp"

class Launcher : public LibXR::Application {
 public:
  enum class TRIGMODE : uint8_t {
    SAFE = 0,
    SINGLE,
    CONTINUE,
    AIM,
  };

  enum class FRICMODE : uint8_t {
    SAFE = 0,
    READY,
  };

  enum class STATE : uint8_t { RESET = 0, FIRE, STOP, JAM };

  typedef struct {
    float heat_limit;
    float heat_cooling;
  } RefereeData;
  struct LauncherParam {
    /*默认弹速*/
    float default_bullet_speed;
    /*摩擦轮半径*/
    float fric_radius;
    /*拨弹盘电机减速比*/
    float trig_gear_ratio;
    /*拨齿数目*/
    uint8_t num_trig_tooth;
    /*弹频*/
    float trig_freq_;
  };

  /**
   * @brief Launcher 构造函数
   *
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param task_stack_depth 任务堆栈深度
   * @param pid_param_trig 拨弹盘PID参数
   * @param pid_param_fric 摩擦轮PID参数
   * @param motor_can1 CAN1上的电机实例（trig）
   * @param motor_can2 CAN2上的电机实例 (fric)
   * @param  min_launch_delay_
   * @param  default_bullet_speed_ 默认弹丸初速度
   * @param  fric_radius_ 摩擦轮半径
   * @param trig_gear_ratio_ 拨弹电机减速比
   * @param  num_trig_tooth_ 拨弹盘中一圈能存储几颗弹丸
   * @param bullet_speed 弹丸速度
   */

  Launcher(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
           uint32_t task_stack_depth,
           LibXR::PID<float>::Param pid_param_trig_angle,
           LibXR::PID<float>::Param pid_param_trig_speed,
           LibXR::PID<float>::Param pid_param_fric, RMMotor *motor_fric_0,
           RMMotor *motor_fric_1, RMMotor *motor_trig,
           LauncherParam launch_param)
      : PARAM(launch_param),
        motor_fric_0_(motor_fric_0),
        motor_fric_1_(motor_fric_1),
        motor_trig_(motor_trig),
        pid_fric_(pid_param_fric),
        pid_trig_sp_(pid_param_trig_speed),
        pid_trig_angle_(pid_param_trig_angle) {
    UNUSED(hw);
    UNUSED(app);
    thread_.Create(this, ThreadFunction, "LauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Launcher *launcher, uint32_t event_id) {
          UNUSED(in_isr);
          launcher->SetMode(event_id);
        },
        this);
    launcher_event_.Register(static_cast<uint32_t>(FRICMODE::SAFE), callback);
    launcher_event_.Register(static_cast<uint32_t>(FRICMODE::READY), callback);
  }

  static void ThreadFunction(Launcher *launcher) {
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> launcher_cmd_tp(
        "launcher_cmd");
    launcher_cmd_tp.StartWaiting();
    while (1) {
      launcher->mutex_.Lock();
      if (launcher_cmd_tp.Available()) {
        launcher->launcher_cmd_ = launcher_cmd_tp.GetData();
        launcher_cmd_tp.StartWaiting();
      }
      launcher->Update();
      launcher->TrigModeSelection();
      launcher->CaclTarget();
      launcher->mutex_.Unlock();

      LibXR::Thread ::Sleep(2);
    }
  }
  /**
   * @brief 更新函数
   *
   */
  void Update() {
    referee_data_.heat_limit = 260.0f;
    referee_data_.heat_cooling = 20.0f;

    motor_fric_0_->Update();
    motor_fric_1_->Update();
    motor_trig_->Update();

    const float LAST_TRIG_MOTOR_ANGLE =
        LibXR::CycleValue<float>(this->motor_trig_->GetAngle());

    motor_trig_->Update();
    /*注意极性*/
    const float DELTA_MOTOR_ANGLE =
        LibXR::CycleValue<float>(this->motor_trig_->GetAngle()) -
        LAST_TRIG_MOTOR_ANGLE;

    last_trig_angle_ += DELTA_MOTOR_ANGLE / PARAM.trig_gear_ratio;
  }

  void TrigModeSelection() {
    auto now = LibXR::Timebase::GetMilliseconds();

    if (launcher_cmd_.isfire && !last_fire_notify_) {
      fire_press_time_ = now;
      press_continue_ = false;
      trig_mod_ = TRIGMODE::SINGLE;
    } else if (launcher_cmd_.isfire && last_fire_notify_) {
      if (!press_continue_ && (now - fire_press_time_ > 200)) {
        press_continue_ = true;
      }
      if (press_continue_) {
        trig_mod_ = TRIGMODE::CONTINUE;
      }
    } else {
      trig_mod_ = TRIGMODE::SAFE;
      press_continue_ = false;
    }

    last_fire_notify_ = launcher_cmd_.isfire;
  }

  void CaclTarget() {
    auto now = LibXR::Timebase::GetMilliseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    switch (fric_mod_) {
      case FRICMODE::SAFE:
        fric_rpm_ = 0.0f;
        break;
      case FRICMODE::READY:
        fric_rpm_ = 4700.0f;
        FricRPMControl(fric_rpm_);
      default:
        break;
    };

    switch (trig_mod_) {
      case TRIGMODE::SAFE: {
        target_trig_angle_ = 0.0f;
        target_trig_rpm_ = 0.0f;
        TrigAngleControl(0.0f);
      } break;
      case TRIGMODE::SINGLE:
        target_trig_angle_ = static_cast<float>(
            (M_2PI / PARAM.num_trig_tooth + last_trig_angle_));
        TrigAngleControl(target_trig_angle_);
        trig_mod_ = TRIGMODE::SAFE;
        break;
      case TRIGMODE::CONTINUE:
        target_trig_angle_ = static_cast<float>(
            (M_2PI / PARAM.num_trig_tooth + last_trig_angle_));
        TrigAngleControl(target_trig_angle_);
        break;
      default:
        break;
    }
  }

  /**
   * @brief 监控函数 (在此应用中未使用)
   *
   */
  void OnMonitor() override {}

  LibXR::Event &GetEvent() { return launcher_event_; }

 private:
  const LauncherParam PARAM;
  RefereeData referee_data_;
  TRIGMODE trig_mod_ = TRIGMODE::SAFE;
  FRICMODE fric_mod_ = FRICMODE::SAFE;

  LibXR::CycleValue<float> target_trig_angle_ = 0.0f;
  float target_trig_rpm_ = 0.0f;
  LibXR::CycleValue<float> last_trig_angle_ = 0.0f;
  float fric_rpm_ = 0.0f;

  RMMotor *motor_fric_0_;
  RMMotor *motor_fric_1_;
  RMMotor *motor_trig_;

  LibXR::PID<float> pid_fric_;
  LibXR::PID<float> pid_trig_sp_;
  LibXR::PID<float> pid_trig_angle_;

  CMD::LauncherCMD launcher_cmd_;

  float dt_ = 0;
  LibXR::MillisecondTimestamp last_online_time_ = 0;

  bool last_fire_notify_ = false;
  bool press_continue_ = false;
  LibXR::MillisecondTimestamp fire_press_time_ = 0;

  LibXR::Thread thread_;
  LibXR::Semaphore semaphore_;
  LibXR::Mutex mutex_;
  LibXR::Event launcher_event_;

  /*---------------------工具函数--------------------------------------------------*/
  void FricRPMControl(float output) {
    float out_left = pid_fric_.Calculate(output, motor_fric_0_->GetRPM(), dt_);
    float out_right =
        -pid_fric_.Calculate(output, motor_fric_1_->GetRPM(), dt_);
    motor_fric_0_->CurrentControl(out_left);
    motor_fric_1_->CurrentControl(out_right);
  }

  void TrigAngleControl(float output) {
    float sp_out = pid_trig_angle_.Calculate(output, last_trig_angle_,
                                             motor_trig_->GetRPM() /
                                                 PARAM.trig_gear_ratio / 60.0f /
                                                 static_cast<float>(M_2PI),
                                             dt_);
    float out = pid_trig_sp_.Calculate(sp_out, motor_trig_->GetRPM(), dt_);
    motor_trig_->CurrentControl(out);
  }
  void SetMode(uint32_t mode) { fric_mod_ = static_cast<FRICMODE>(mode); }
};
