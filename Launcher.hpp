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
  - motor_can1: '@motor_can1'
  - motor_can2: '@motor_can2'
  - num_trig_tooth: 0.0
  - trig_gear_ratio: 0.0
  - fric_radius: 0.0
  - cover_open_duty: 0.0
  - cover_close_duty: 0.0
  - default_bullet_speed: 0.0
  - min_launch_delay: 0.0
template_args:
  - MotorType: RMMotorContainer
required_hardware:
  - cmd
  - motor_can1
  - motor_can2
depends:

=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "CMD.hpp"
#include "Eigen/Core"
#include "Motor.hpp"
#include "app_framework.hpp"
#include "pid.hpp"
#define LAUNCHER_TRIG_SPEED_MAX (16000.0f)

template <typename MotorType>
class Launcher : public LibXR::Application {
 public:
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
           LibXR::PID<float>::Param pid_param_fric,
           Motor<MotorType> &motor_can1, Motor<MotorType> &motor_can2,
           float num_trig_tooth, float trig_gear_ratio, float fric_radius,
           float cover_open_duty, float cover_close_duty,
           float default_bullet_speed, uint32_t min_launch_delaye)
      : motor_can2_(motor_can2),
        motor_can1_(motor_can1),
        pid_fric_(pid_param_fric),
        pid_trig_(pid_param_trig),
        cmd_(cmd)

  {
    thread_.Create(this, ThreadFunction, "LauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);

    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");
  }

  static void ThreadFunction(Launcher *launcher) {
    auto now_ = LibXR::Timebase::GetMicroseconds();
    launcher->dt_ = (now_ - launcher->last_online_time_);
    launcher->last_online_time_ = now_;

    while (1) {
      launcher->semaphore_.Wait(UINT32_MAX);
      launcher->Update();
      launcher->semaphore_.Post();
      launcher->Control();
      launcher->SelfResolution();

      //event_active 接受dr16 mode转变
    }
  }

  void Update() {
    motor_can1_.Update(5);
    motor_can2_.Update(2);
    motor_can2_.Update(3);
  }

  void SelfResolution() {
    prev_omega_fric_ = now_omega_fric_;
    prev_omega_trig1_ = now_omega_trig1_;
    prev_omega_trig2_ = now_omega_trig2_;

    now_angle_trig_ = motor_can1_.GetAngle(5);
    now_angle_fric1_ = motor_can2_.GetAngle(2);
    now_angle_fric2_ = motor_can2_.GetAngle(3);
    now_omega_fric_ = motor_can1_.GetSpeed(5);
    now_omega_trig1_ = motor_can2_.GetSpeed(2);
    now_angle_fric2_ = motor_can2_.GetSpeed(3);
  }

  void Control() {
    const float output_trig_ = std::clamp(
        pid_trig_.Calculate(target_angle_trig_,
                            now_angle_trig_ / LAUNCHER_TRIG_SPEED_MAX, dt_),
        0.0f, LAUNCHER_TRIG_SPEED_MAX);

    float output_fric1_ = bullet_speed_to_fric_rpm(0, fric_radius_, 1);
    float output_fric2_ = -output_fric1_;

    motor_can1_.SetCurrent(5, output_trig_);
    motor_can2_.SetCurrent(2, output_fric1_);
    motor_can2_.SetCurrent(3, output_fric2_);
  }

  float bullet_speed_to_fric_rpm(float bullet_speed, float fric_radius,
                                 bool is17mm) {
    if (bullet_speed == 0.0f) {
      return 0.f;
    }
    if (is17mm) {
      if (bullet_speed == 15.0f) {
        return 4670.f;
      }
      if (bullet_speed == 18.0f) {
        return 5200.f;
      }
      if (bullet_speed == 25.0f) {
        return 7400.f;
      }
    } else {
      if (bullet_speed == 10.0f) {
        return 4450.f;
      }
      if (bullet_speed == 16.0f) {
        return 5700.f;
      }
    }
  }

    /**
   * @brief 监控函数 (在此应用中未使用)
   *
   */
  void OnMonitor() override {}

 private:
  // 最小发射间隔
  uint32_t min_launch_delay_ = 0.0f;
  // 默认弹丸初速度
  float default_bullet_speed_ = 0.0f;
  // 摩擦轮半径
  float fric_radius_ = 0.0f;
  // 拨弹电机减速比
  float trig_gear_ratio_ = 0.0f;
  // 拨弹盘中一圈能存储几颗弹丸
  float num_trig_tooth_ = 0.0f;

  float now_angle_trig_ = 0.0f;
  float now_angle_fric1_ = 0.0f;
  float now_angle_fric2_ = 0.0f;

  float prev_omega_fric_ = 0.0f;
  float prev_omega_trig1_ = 0.0f;
  float prev_omega_trig2_ = 0.0f;

  float now_omega_fric_ = 0.0f;
  float now_omega_trig1_ = 0.0f;
  float now_omega_trig2_ = 0.0f;

  float target_angle_trig_ = M_2PI / num_trig_tooth_;
  float target_angle_fric_ = 0.0f;
  // 摩擦轮转速
  float fric_rpm_ = 0.0f;
  //! CAN2上的电机实例 (fric)
  Motor<MotorType> &motor_can2_;
  //! CAN1上的电机实例 (trig)
  Motor<MotorType> &motor_can1_;

  LibXR::PID<float> pid_fric_;
  LibXR::PID<float> pid_trig_;
  //! 时间间隔 (s)
  LibXR::MicrosecondTimestamp::Duration dt_ = 0;
  //! 上次在线时间
  LibXR::MicrosecondTimestamp last_online_time_ = 0;
  //! 当前时间
  LibXR::MicrosecondTimestamp now_ = 0;
  //! 命令模块实例
  CMD &cmd_;
  //! 发射控制线程
  LibXR::Thread thread_;

  LibXR::Semaphore semaphore_;
};
