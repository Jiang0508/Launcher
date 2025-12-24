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
  - pid_param_fric_0:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_param_fric_1:
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



template <int FRIC_NUM, int TRIG_NUM>

class InfantryLauncher {
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
    float num_trig_tooth;
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

  InfantryLauncher(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
           std::array<RMMotor*, FRIC_NUM> motor_fric,
           std::array<RMMotor*, TRIG_NUM> motor_trig,
           uint32_t task_stack_depth,
           LibXR::PID<float>::Param pid_param_trig_angle,
           LibXR::PID<float>::Param pid_param_trig_speed,
           LibXR::PID<float>::Param pid_param_fric_0,
           LibXR::PID<float>::Param pid_param_fric_1,
           LauncherParam launch_param)
      : PARAM(launch_param),
        motor_fric_(motor_fric),
        motor_trig_(motor_trig),
        pid_fric_0_(pid_param_fric_0),
        pid_fric_1_(pid_param_fric_1),
        pid_trig_sp_(pid_param_trig_speed),
        pid_trig_angle_(pid_param_trig_angle) {
    UNUSED(hw);
    UNUSED(app);
    thread_.Create(this, ThreadFunction, "LauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);

    if (PARAM.trig_freq_ > 0.0f) {
      min_launch_delay_ = 1.0f / PARAM.trig_freq_;
    } else {
      min_launch_delay_ = 0.0f;
    }
  }

  static void ThreadFunction(InfantryLauncher *launcher) {
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

    static float last_motor_angle = 0.0f;
    static bool initialized = false;

    motor_fric_[0]->Update();
    motor_fric_[1]->Update();
    motor_trig_->Update();

    float current_motor_angle = motor_trig_[0]->GetAngle();

    if (!initialized) {
      last_motor_angle = current_motor_angle;
      initialized = true;
      return;
    }

    float last_angle = last_motor_angle;
    float current_angle = current_motor_angle;

    float delta_trig_angle = LibXR::CycleValue<float>(current_angle) -
                             LibXR::CycleValue<float>(last_angle);
    if (!motor_trig_->GetReverse()) {
      trig_angle_ += delta_trig_angle / PARAM.trig_gear_ratio;
    } else {
      trig_angle_ -= delta_trig_angle / PARAM.trig_gear_ratio;
    }
    last_motor_angle = current_motor_angle;
  }

  void TrigModeSelection() {
    auto now = LibXR::Timebase::GetMilliseconds();

    if (launcher_cmd_.isfire) {
      if (!last_fire_notify_) {
        fire_press_time_ = now;
        press_continue_ = false;
        trig_mod_ = TRIGMODE::SINGLE;
      } else {
        if (!press_continue_ && (now - fire_press_time_ > 500)) {
          press_continue_ = true;
        }
        if (press_continue_) {
          trig_mod_ = TRIGMODE::CONTINUE;
        }
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
        FricRPMControl(fric_rpm_);
        break;
      case FRICMODE::READY:
        fric_rpm_ = 3000.0f;
        FricRPMControl(fric_rpm_);
        break;
      default:
        break;
    }

    static TRIGMODE last_trig_mod = TRIGMODE::SAFE;

    if (last_trig_mod == TRIGMODE::SAFE && trig_mod_ == TRIGMODE::SINGLE) {
      target_trig_angle_ =
          static_cast<float>(M_2PI / PARAM.num_trig_tooth + trig_angle_);
    }

    switch (trig_mod_) {
      case TRIGMODE::SAFE: {
        TrigAngleControl(trig_angle_);
      } break;

      case TRIGMODE::SINGLE: {
        TrigAngleControl(target_trig_angle_);

        float angle_err = static_cast<float>(target_trig_angle_ - trig_angle_);
        if (std::fabs(angle_err) == 0.0f) {
          trig_mod_ = TRIGMODE::SAFE;
        }
      } break;

      case TRIGMODE::CONTINUE: {
        float since_last = (now - last_trig_time_).ToSecondf();
        if (PARAM.trig_freq_ > 0.0f) {
          float trig_speed = 1.0f / PARAM.trig_freq_;
          if (since_last >= trig_speed) {
            target_trig_angle_ +=
                static_cast<float>(M_2PI / PARAM.num_trig_tooth);
            last_trig_time_ = now;
          }
        }
        TrigAngleControl(target_trig_angle_);
      } break;

      default:
        break;
    }

    last_trig_mod = trig_mod_;
  }


 private:
  const LauncherParam PARAM;
  RefereeData referee_data_;
  TRIGMODE trig_mod_ = TRIGMODE::SAFE;
  FRICMODE fric_mod_ = FRICMODE::SAFE;

  float target_trig_angle_ = 0.0f;
  float trig_angle_ = 0.0f;
  float fric_rpm_ = 0.0f;
  float min_launch_delay_ = 0.0f;
  LibXR::MillisecondTimestamp last_trig_time_ = 0;

  std::array<RMMotor *, FRIC_NUM> motor_fric_;
  std::array<RMMotor *, TRIG_NUM> motor_trig_;

  LibXR::PID<float> pid_fric_0_;
  LibXR::PID<float> pid_fric_1_;
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
  uint32_t launcher_event_ = 0;

  /*---------------------工具函数--------------------------------------------------*/
  void FricRPMControl(float output) {
    float out_left =
        pid_fric_0_.Calculate(output, motor_fric_[0]->GetRPM(), dt_);
    float out_right =
        pid_fric_1_.Calculate(output, motor_fric_[1]->GetRPM(), dt_);
    motor_fric_[0]->CurrentControl(out_left);
    motor_fric_[1]->CurrentControl(out_right);
  }

  void TrigAngleControl(float target_angle) {
    float plate_omega_ref = pid_trig_angle_.Calculate(
        target_angle, trig_angle_,
        motor_trig_[0]->GetOmega() / PARAM.trig_gear_ratio, dt_);

    float motor_omega_ref = plate_omega_ref;
    float out = pid_trig_sp_.Calculate(
        motor_omega_ref, motor_trig_->GetOmega() / PARAM.trig_gear_ratio, dt_);

    motor_trig_[0]->CurrentControl(out);
  }
  /**
   * @brief 设置底盘模式 (由 Launcher 外壳调用)
   * @param mode 要设置的新模式
   */
  void SetMode(uint32_t mode) { launcher_event_ = mode;}
};
