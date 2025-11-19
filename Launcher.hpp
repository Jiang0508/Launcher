#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - task_stack_depth: 2048
  - pid_param_trig:
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
  - motor_fric_0: '@motor_fric_0'
  - motor_fric_1: '@motor_fric_1'
  - motor_trig: '@motor_trig'
  - launcher_param:
      min_launch_delay: 0.0
      default_bullet_speed: 0.0
      fric_radius: 0.0
      trig_gear_ratio: 0.0
      num_trig_tooth: 0
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
#include "pid.hpp"
#include "thread.hpp"
#define LAUNCHER_TRIG_SPEED_MAX (16000.0f)

template <typename MotorType>
class Launcher : public LibXR::Application {
  enum class MOD : uint8_t {
    SAFE = 0,
    SINGLE,
    BREAK_OUT,
    AIM,
  };

  enum class STATE : uint8_t { RESET = 0, FIRE, STOP, JAM };

  typedef struct {
    float heat_limit;
    float heat_dissipation;
  } RefereeData;

 public:
  struct LauncherParam {
    float min_launch_delay;
    float default_bullet_speed;
    float fric_radius;
    float trig_gear_ratio;
    uint8_t num_trig_tooth;
  };

  /**
   * @brief Launcher 构造函数
   *
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param cmd 命令模块实例
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
           CMD &cmd, uint32_t task_stack_depth,
           LibXR::PID<float>::Param pid_param_trig,
           LibXR::PID<float>::Param pid_param_fric, RMMotor *motor_fric_0,
           RMMotor *motor_fric_1, RMMotor *motor_trig,
           LauncherParam launch_param)
      : PARAM(launch_param),
        motor_fric_0_(motor_fric_0),
        motor_fric_1_(motor_fric_1),
        motor_trig_(motor_trig),
        pid_fric_(pid_param_fric),
        pid_trig_(pid_param_trig),
        cmd_(cmd) {
    UNUSED(hw);
    UNUSED(app);
    thread_.Create(this, ThreadFunction, "LauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }

  static void ThreadFunction(Launcher *launcher) {
    auto last_time = LibXR::Timebase::GetMilliseconds();
    while (1) {
      launcher->Update();
      LibXR::Thread ::SleepUntil(last_time, 2);
    }
  }
  /**
   * @brief 更新函数
   *
   */
  void Update() {
    auto now = LibXR::Timebase::GetMilliseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    referee_data_.heat_limit = 260.0f;
    referee_data_.heat_dissipation = 20.0f;

    motor_fric_0_->Update();
    motor_fric_0_->Update();
    motor_trig_->Update();
  }

  void ModeSelection(float default_bullet_speed_, bool is17mm) {
    switch (mod_) {
      case MOD::SAFE:
        now_mod_ = MOD::SAFE;
        break;
      case MOD::SINGLE:
        now_mod_ = MOD::SINGLE;
        break;
      case MOD::BREAK_OUT:
      case MOD::AIM:
        now_mod_ = MOD::BREAK_OUT;
        break;
      default:
        break;
    }

    last_mod_ = now_mod_;
    if (last_mod_ == MOD::SAFE && now_mod_ == MOD::SINGLE) {
      fric_rpm_ = BulletSpeedToFricRpm(default_bullet_speed_, is17mm);
      target_trig_angle_ = (M_2PI / PARAM.num_trig_tooth + last_trig_angle_);
      target_trig_rpm_ = 0.0f;
      last_trig_angle_ = target_trig_angle_;
    } else if (last_mod_ == MOD::SAFE && now_mod_ == MOD::BREAK_OUT) {
      fric_rpm_ = BulletSpeedToFricRpm(default_bullet_speed_, is17mm);
      target_trig_rpm_ = 0.0f;
      target_trig_angle_ = 0.0f;
    }
  }

  void SelfResolution() {}

  void BehavioralJudgment() {}

  void OutputToDynamics() {}

  float BulletSpeedToFricRpm(float bullet_speed, bool is17mm) {
    if (bullet_speed == 0.0f) {
      return 0.0f;
    } else if (bullet_speed > 0.0f) {
      if (is17mm) {
        if (bullet_speed == 15.0f) {
          return 4670.0f;
        }
        if (bullet_speed == 18.0f) {
          return 5200.0f;
        }
        if (bullet_speed == 25.0f) {
          return 7400.0f;
        }
      } else {
        if (bullet_speed == 10.0f) {
          return 4450.0f;
        }
        if (bullet_speed == 16.0f) {
          return 5700.0f;
        }
      }
    }
    return 0.0f;
  }
  /**
   * @brief 监控函数 (在此应用中未使用)
   *
   */
  void OnMonitor() override {}

 private:
  const LauncherParam PARAM;
  RefereeData referee_data_;
  MOD mod_ = MOD::SAFE;
  STATE state_ = STATE::STOP;

  uint8_t now_mod_ = 0;
  uint8_t last_mod_ = 0;

  float calorie_remain_ = 0.0f;

  float target_trig_angle_ = 0.0f;
  float target_trig_rpm_ = 0.0f;
  float last_trig_angle_ = 0.0f;
  float fric_rpm_ = 0.0f;

  bool is_reset_ = false;

  typename MotorType::RMMotor *motor_fric_0_;
  typename MotorType::RMMotor *motor_fric_1_;
  typename MotorType::RMMotor *motor_trig_;

  LibXR::PID<float> pid_fric_;
  LibXR::PID<float> pid_trig_;

  float dt_ = 0;
  LibXR::MillisecondTimestamp last_online_time_ = 0;

  CMD &cmd_;

  LibXR::Thread thread_;

  LibXR::Semaphore semaphore_;
};
