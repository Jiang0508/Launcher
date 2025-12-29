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
  - FRIC_NUM: 4
  - TRIG_NUM: 1
required_hardware:
  - dr16
  - can
depends:
  - qdu-future/CMD
  - qdu-future/RMMotor
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstdint>

#include "CMD.hpp"
#include "HeroLauncher.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"


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

typedef struct {
  float heat_limit;
  float heat_cooling;
  float cooling_rate;
  uint8_t level;
} RefereeData;

struct HeatControl {
  float heat;          /* 现在热量水平 */
  float last_heat;     /* 之前的热量水平 */
  float heat_limit;    /* 热量上限 */
  float speed_limit;   /* 弹丸初速是上限 */
  float cooling_rate;  /* 冷却速率 */
  float heat_increase; /* 每发热量增加值 */

  uint8_t cooling_acc; //冷却增益

  uint32_t available_shot; /* 热量范围内还可以发射的数量 */
};

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

template<int FRIC_NUM, int TRIG_NUM>
class HeroLauncher {
 public:
  HeroLauncher(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
               std::array<RMMotor *,FRIC_NUM> motor_fric,
               std::array<RMMotor *,TRIG_NUM> motor_trig,
               uint32_t task_stack_depth,
               LibXR::PID<float>::Param trig_angle_pid,
               LibXR::PID<float>::Param trig_speed_pid,
               LibXR::PID<float>::Param fric_speed_pid_0,
               LibXR::PID<float>::Param fric_speed_pid_1,
               LibXR::PID<float>::Param fric_speed_pid_2,
               LibXR::PID<float>::Param fric_speed_pid_3,
               LauncherParam launcher_param, CMD *cmd)
      : PARAM(launcher_param),
        motor_fric_(motor_fric),
        motor_trig_(motor_trig),
        trig_angle_pid_(trig_angle_pid),
        trig_speed_pid_(trig_speed_pid),
        fric_speed_pid_{fric_speed_pid_0, fric_speed_pid_1, fric_speed_pid_2,
                        fric_speed_pid_3},
        cmd_(cmd) {
    UNUSED(hw);
    UNUSED(app);
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");

      motor_fric_front_left_ = motor_fric_[0];
      motor_fric_front_right_ = motor_fric_[1];
      motor_fric_back_left_ = motor_fric_[2];
      motor_fric_back_right_ = motor_fric_[3];


    thread_.Create(this, ThreadFunction, "HeroLauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, HeroLauncher *HeroLauncher, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          HeroLauncher->LostCtrl();
        },
        this);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
  }

  static void ThreadFunction(HeroLauncher *HeroLauncher) {
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> launcher_cmd_tp(
        "launcher_cmd");
    launcher_cmd_tp.StartWaiting();
    while (true) {
      LibXR::MillisecondTimestamp last_time =
          LibXR::Timebase::GetMilliseconds();

      if (launcher_cmd_tp.Available()) {
        HeroLauncher->launcher_cmd_ = launcher_cmd_tp.GetData();
        launcher_cmd_tp.StartWaiting();
      }
      HeroLauncher->mutex_.Lock();
      HeroLauncher->Update();
      HeroLauncher->HeatLimit();
      HeroLauncher->Control();

      HeroLauncher->mutex_.Unlock();
      HeroLauncher->thread_.SleepUntil(last_time, 2);
    }
  }

  void Update() {

    auto now = LibXR::Timebase::GetMicroseconds();
    this->dt_ = (now - this->last_wakeup_).ToSecondf();
    this->last_wakeup_ = now;

    const float LAST_TRIG_MOTOR_ANGLE =
        LibXR::CycleValue<float>(this->motor_trig_[0]->GetAngle());
    motor_fric_front_left_->Update();
    motor_fric_front_right_->Update();
    motor_fric_back_left_->Update();
    motor_fric_back_right_->Update();
    motor_trig_[0]->Update();
    const float DELTA_MOTOR_ANGLE =
        LibXR::CycleValue<float>(this->motor_trig_[0]->GetAngle()) -
        LAST_TRIG_MOTOR_ANGLE;
    this->trig_angle_ += DELTA_MOTOR_ANGLE / PARAM.trig_gear_ratio;
  }

  void Control() {
    LibXR::MillisecondTimestamp now_time = LibXR::Timebase::GetMilliseconds();

    now_ = LibXR::Timebase::GetMilliseconds();

    if (launcher_cmd_.isfire && !last_fire_notify_) {  //拨弹盘模式设定
      fire_press_time_ = now_time;
      press_continue_ = false;
      trig_mod_ = TRIGMODE::SINGLE;
    } else if (launcher_cmd_.isfire && last_fire_notify_) {
      if (!press_continue_ && (now_time - fire_press_time_ > 200)) {
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

    switch (launcher_event_) {
      case static_cast<uint32_t>(FRICMODE::SAFE):
        fric_target_speed_[0] = 0;
        fric_target_speed_[1] = 0;
        fric_target_speed_[2] = 0;
        fric_target_speed_[3] = 0;

        motor_fric_front_left_->Relax();
        motor_fric_front_right_->Relax();
        motor_fric_back_left_->Relax();
        motor_fric_back_right_->Relax();
        break;
      case static_cast<uint32_t>(FRICMODE::READY):

        fric_target_speed_[0] = PARAM.fric2_setpoint_speed;
        fric_target_speed_[1] = PARAM.fric2_setpoint_speed;
        fric_target_speed_[2] = PARAM.fric1_setpoint_speed;
        fric_target_speed_[3] = PARAM.fric1_setpoint_speed;
        break;
      default:
        break;
    }

    current_back_left_ = motor_fric_front_left_->GetCurrent();

    if (first_loading_) {  //首次发弹进行弹丸位置标定
      if (trig_mod_ == TRIGMODE::SINGLE) {
        fire_flag_ = true;
      }
      if (fire_flag_) {
        if (start_loading_time_ == 0) {
          start_loading_time_ = LibXR::Timebase::GetMilliseconds();
        }

        trig_setpoint_angle_ -= M_2PI / 1002.0f;
        last_change_angle_time_ = LibXR::Timebase::GetMilliseconds();

        delay_time_++;
      }

      if (delay_time_ > 50) {//延迟50个控制周期
        if (std::abs(motor_fric_back_left_->GetCurrent()) > 5) {  // 发弹检测
          trig_zero_angle_ = trig_angle_;  // 获取电机当前位置
          trig_setpoint_angle_ = trig_angle_ - M_2PI / 7.35f;  // 偏移量

          fire_flag_ = false;
          first_loading_ = false;
          fired_++;

          mark_launch_ = true;
        }
      }

    } else { //常规发弹逻辑
      if (trig_mod_ == TRIGMODE::SINGLE) {
        mark_launch_ = false;
        if (!enable_fire_) {
          if (heat_ctrl_.available_shot) {
            trig_setpoint_angle_ -= M_2PI / 6.0f;

            enable_fire_ = true;
            mark_launch_ = false;
            start_fire_time_ = LibXR::Timebase::GetMilliseconds();

            trig_mod_ = TRIGMODE::SAFE;
          }
        }
      }

      // 添加发射超时检测（超过100毫秒未检测到发弹则重置状态）
      if (start_fire_time_ > 0 && (now_ - start_fire_time_ > 100) &&
          !mark_launch_) {
        fire_flag_ = false;
        //first_loading_ = false;
        enable_fire_ = false;
        //start_loading_time_ = 0;
        //delay_time_ = 0;
        start_fire_time_ = now_;
      }

      if (!mark_launch_) { //发弹状态检测
        {
          if (std::abs(motor_fric_back_left_->GetCurrent()) > 5) {
            fire_flag_ = false;

            fired_++;

            mark_launch_ = true;
            enable_fire_ = false;
            finish_fire_time_ = LibXR::Timebase::GetMilliseconds();
          }
        }
      }
    }
    real_launch_delay_ = (finish_fire_time_ - start_fire_time_).ToSecondf();

    fric_output_[0] = fric_speed_pid_[0].Calculate(
        fric_target_speed_[0], motor_fric_front_left_->GetRPM(), dt_);
    fric_output_[1] = fric_speed_pid_[1].Calculate(
        fric_target_speed_[1], motor_fric_front_right_->GetRPM(), dt_);
    fric_output_[2] = fric_speed_pid_[2].Calculate(
        fric_target_speed_[2], motor_fric_back_left_->GetRPM(), dt_);
    fric_output_[3] = fric_speed_pid_[3].Calculate(
        fric_target_speed_[3], motor_fric_back_right_->GetRPM(), dt_);

    motor_fric_front_left_->CurrentControl(fric_output_[0]);
    motor_fric_front_right_->CurrentControl(fric_output_[1]);
    motor_fric_back_left_->CurrentControl(fric_output_[2]);
    motor_fric_back_right_->CurrentControl(fric_output_[3]);

    trig_setpoint_speed_ =
        trig_angle_pid_.Calculate(trig_setpoint_angle_, trig_angle_, dt_);

    trig_output_ = trig_speed_pid_.Calculate(trig_setpoint_speed_,
                                             motor_trig_[0]->GetRPM(), dt_);

    motor_trig_[0]->CurrentControl(trig_output_);
  }
  void HeatLimit() {
    heat_ctrl_.heat_limit = referee_data_.heat_limit;
    heat_ctrl_.heat_limit = 1000;// for debug
    heat_ctrl_.heat_increase = 100.0f;
    heat_ctrl_.cooling_rate = referee_data_.cooling_rate;
    heat_ctrl_.cooling_rate = 1000;  // for debug
    heat_ctrl_.heat -=
        heat_ctrl_.cooling_rate / 500.0;  // 每个控制周期的冷却恢复
    if (fired_ >= 1) {
      heat_ctrl_.heat += heat_ctrl_.heat_increase;
      fired_ = 0;
    }
    heat_ctrl_.heat = std::clamp(heat_ctrl_.heat, 0.0f, heat_ctrl_.heat_limit);
    float available_float =
        (this->heat_ctrl_.heat_limit - this->heat_ctrl_.heat) /
        this->heat_ctrl_.heat_increase;
    heat_ctrl_.available_shot = static_cast<uint32_t>(available_float);
  }
  void LostCtrl() {
    // 重置所有发射相关的状态变量到初始模式
    launcher_event_ = static_cast<uint32_t>(FRICMODE::SAFE);
    trig_mod_ = TRIGMODE::SAFE;

    // 重置发射控制标志
    fire_flag_ = false;
    enable_fire_ = false;
    mark_launch_ = false;
    first_loading_ = true;
    press_continue_ = false;

    // 重置计数器
    fired_ = 0;
    delay_time_ = 0;

    // 重置时间戳
    fire_press_time_ = 0;
    start_fire_time_ = 0;
    finish_fire_time_ = 0;
    start_loading_time_ = 0;
    last_change_angle_time_ = 0;

    // 重置角度相关变量
    trig_zero_angle_ = 0.0f;
    trig_angle_ = 0.0f;
    trig_setpoint_angle_ = 0.0f;

    // 重置速度目标值
    fric_target_speed_[0] = 0.0f;
    fric_target_speed_[1] = 0.0f;
    fric_target_speed_[2] = 0.0f;
    fric_target_speed_[3] = 0.0f;

    // 重置发射命令
    launcher_cmd_.isfire = false;
    last_fire_notify_ = false;

    // 重置延迟计算
    real_launch_delay_ = 0.0f;
  }

  void SetMode(uint32_t mode) { launcher_event_ = mode; }

 private:
  const LauncherParam PARAM;
  RefereeData referee_data_;
  TRIGMODE trig_mod_ = TRIGMODE::SAFE;

  HeatControl heat_ctrl_;

  bool first_loading_ = true;

  float dt_ = 0.0f;

  LibXR::MillisecondTimestamp now_ = 0;

  LibXR::MicrosecondTimestamp last_wakeup_;

  LibXR::MillisecondTimestamp last_change_angle_time_ = 0;

  LibXR::MillisecondTimestamp start_loading_time_ = 0;

  RMMotor *motor_fric_front_left_;
  RMMotor *motor_fric_front_right_;
  RMMotor *motor_fric_back_left_;
  RMMotor *motor_fric_back_right_;

  std::array<RMMotor*, FRIC_NUM> motor_fric_;
  std::array<RMMotor *, TRIG_NUM> motor_trig_;


  float trig_setpoint_angle_ = 0.0f;
  float trig_setpoint_speed_ = 0.0f;

  float trig_zero_angle_ = 0.0f;
  float trig_angle_ = 0.0f;
  float trig_output_ = 0.0f;

  float fric_target_speed_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  float fric_output_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  LibXR::PID<float> trig_angle_pid_;
  LibXR::PID<float> trig_speed_pid_;

  LibXR::PID<float> fric_speed_pid_[4] = {
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param())};

  CMD::LauncherCMD launcher_cmd_;

  float current_back_left_ = 0.0f;

  bool fire_flag_ = false; //发射命令标志位

  uint8_t fired_ = 0;//已发射弹丸

  bool enable_fire_ = false; //拨弹盘旋转命令发出标志位
  bool mark_launch_ = false; //拨弹发射完成标志位

  LibXR::MillisecondTimestamp start_fire_time_ = 0;
  LibXR::MillisecondTimestamp finish_fire_time_ = 0;
  float real_launch_delay_ = 0.0f;

  bool last_fire_notify_ = false;
  bool press_continue_ = false;
  LibXR::MillisecondTimestamp fire_press_time_ = 0;

  uint8_t delay_time_ = 0;

  CMD *cmd_;
  LibXR::Thread thread_;
  LibXR::Semaphore semaphore_;
  LibXR::Mutex mutex_;
  uint32_t launcher_event_;
};
