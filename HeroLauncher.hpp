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

#include <algorithm>
#include <cstdint>

#include "CMD.hpp"
#include "Motor.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "timebase.hpp"

#ifdef DEBUG
#include "DebugCore.hpp"
#include "ramfs.hpp"
#endif

/**
 * @brief 英雄发射机构实现
 * @details 负责摩擦轮、拨弹盘控制与热量约束发射逻辑。
 *          作为 Launcher<HeroLauncher> 的内部逻辑类，不拥有线程和事件注册。
 */
class HeroLauncher {
 public:
  static constexpr int FRIC_NUM = 4;
  static constexpr float TRIG_ZERO_ANGLE_OFFSET = 0.90f;
  static constexpr float TRIG_LOADING_ANGLE_STEP =
      static_cast<float>(M_2PI) / 1000.0f;

  enum class TrigMode : uint8_t {
    RELAX = 0,
    SAFE,
    SINGLE,
    CONTINUE,
  };

  enum class LauncherEvent : uint8_t {
    SET_FRICMODE_RELAX,
    SET_FRICMODE_SAFE,
    SET_FRICMODE_READY,
  };
  struct HeatControl {
    float heat;          /* 现在热量水平 */
    float last_heat;     /* 之前的热量水平 */
    float heat_limit;    /* 热量上限 */
    float speed_limit;   /* 弹丸初速上限 */
    float cooling_rate;  /* 冷却速率 */
    float heat_increase; /* 每发热量增加值 */

    uint8_t cooling_acc;  // 冷却增益

    uint32_t available_shot; /* 热量范围内还可以发射的数量 */
  };

  struct LauncherParam {
    /*一级摩擦轮转速*/
    float fric1_setpoint_speed;
    /*二级摩擦轮转速*/
    float fric2_setpoint_speed;
    /*拨弹盘电机减速比*/
    float trig_gear_ratio;
    /*拨齿数目*/
    uint8_t num_trig_tooth;
    /*弹频*/
    float trig_freq;
    uint8_t fric_num;
  };

  /**
   * @brief 构造 HeroLauncher
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param motor_fric_front_left 前左摩擦轮电机
   * @param motor_fric_front_right 前右摩擦轮电机
   * @param motor_fric_back_left 后左摩擦轮电机
   * @param motor_fric_back_right 后右摩擦轮电机
   * @param motor_trig 拨弹电机
   * @param task_stack_depth 线程栈深（由外壳使用）
   * @param trig_angle_pid 拨弹角度环参数
   * @param trig_speed_pid 拨弹速度环参数
   * @param fric_speed_pid_0~3 摩擦轮速度环参数
   * @param launcher_param 发射器参数
   * @param cmd CMD 模块指针
   */
  HeroLauncher(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
               RMMotor* motor_fric_front_left, RMMotor* motor_fric_front_right,
               RMMotor* motor_fric_back_left, RMMotor* motor_fric_back_right,
               RMMotor* motor_trig, uint32_t task_stack_depth,
               LibXR::PID<float>::Param trig_angle_pid,
               LibXR::PID<float>::Param trig_speed_pid,
               LibXR::PID<float>::Param fric_speed_pid_0,
               LibXR::PID<float>::Param fric_speed_pid_1,
               LibXR::PID<float>::Param fric_speed_pid_2,
               LibXR::PID<float>::Param fric_speed_pid_3,
               LauncherParam launcher_param, CMD* cmd)
      : param_(launcher_param),
        motor_fric_front_left_(motor_fric_front_left),
        motor_fric_front_right_(motor_fric_front_right),
        motor_fric_back_left_(motor_fric_back_left),
        motor_fric_back_right_(motor_fric_back_right),
        motor_trig_(motor_trig),
        trig_angle_pid_(trig_angle_pid),
        trig_speed_pid_(trig_speed_pid),
        fric_speed_pid_{fric_speed_pid_0, fric_speed_pid_1, fric_speed_pid_2,
                        fric_speed_pid_3} {
    UNUSED(hw);
    UNUSED(app);
    UNUSED(task_stack_depth);
    UNUSED(cmd);

    last_wakeup_ = LibXR::Timebase::GetMicroseconds();
  }

  /**
   * @brief 更新电机反馈和状态量
   */
  void Update() {
    this->last_wakeup_ = LibXR::Timebase::GetMicroseconds();

    const float LAST_TRIG_MOTOR_ANGLE =
        LibXR::CycleValue<float>(param_trig_.abs_angle);

    motor_fric_front_left_->Update();
    motor_fric_front_right_->Update();
    motor_fric_back_left_->Update();
    motor_fric_back_right_->Update();
    motor_state_ = motor_trig_->Update();

    param_motor_fric_front_left_ = motor_fric_front_left_->GetFeedback();
    param_motor_fric_front_right_ = motor_fric_front_right_->GetFeedback();
    param_motor_fric_back_left_ = motor_fric_back_left_->GetFeedback();
    param_motor_fric_back_right_ = motor_fric_back_right_->GetFeedback();
    param_trig_ = motor_trig_->GetFeedback();
    const float DELTA_MOTOR_ANGLE =
        LibXR::CycleValue<float>(param_trig_.abs_angle) - LAST_TRIG_MOTOR_ANGLE;
    this->trig_angle_ += DELTA_MOTOR_ANGLE / param_.trig_gear_ratio;
  }

  /**
   * @brief 状态机与热量计算
   * @details 更新热量限制，更新拨弹状态机。
   */
  void Solve() {
    SoftStart();
    HeatLimit();
    UpdateTrigMode();
    UpdateFricTarget();
  }

  /**
   * @brief 控制输出
   * @details 拨弹控制、发弹检测和摩擦轮PID输出。
   */
  void Control() {
    if (motor_state_ == LibXR::ErrorCode::OK) {
      if (first_loading_) {
        FirstLoadingControl();
      } else {
        NormalFireControl();
      }
      real_launch_delay_ =
          (finish_fire_time_ - start_fire_time_).ToMillisecond();
      FricPidControl();
      TrigPidControl();
    } else {
      Reset();
    }
  }

  /**
   * @brief 设置发射器模式
   * @param mode 事件ID，对应 LauncherEvent
   */
  void SetMode(uint32_t mode) {
    launcher_event_ = static_cast<LauncherEvent>(mode);
  }

  /**
   * @brief 失控处理
   */
  void LostCtrl() {
    // 重置所有发射相关的状态变量到初始模式
    launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
    trig_mode_ = TrigMode::RELAX;

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

    trig_output_ = 0.0f;

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

    soft_start_finish_ = false;
  }
  /**
   * @brief 重置发射器状态
   * @details 将所有发射相关的状态变量、标志位、时间戳和角度值重置到初始状态。
   */
  void Reset() {
    first_loading_ = true;
    fire_flag_ = false;
    enable_fire_ = false;
    mark_launch_ = false;
    press_continue_ = false;

    fired_ = 0;

    start_fire_time_ = 0;
    finish_fire_time_ = 0;

    launcher_cmd_.isfire = false;
    last_fire_notify_ = false;

    trig_angle_ = 0.0f;
    trig_zero_angle_ = 0.0f;
    trig_setpoint_angle_ = 0.0f;

    delay_time_ = 0;

    soft_start_finish_ = false;
  }

  void OnMonitor() {}

  void SetControlDt(float dt) { dt_ = dt; }

  /**
   * @brief 调试命令入口
   */
#ifdef DEBUG
  int DebugCommand(int argc, char** argv);
#endif

  /* 外壳可直接写入的命令数据 */
  CMD::LauncherCMD launcher_cmd_;  // NOLINT
  Referee::LauncherPack ref_data_{};

 private:
  LauncherParam param_;

  TrigMode trig_mode_ = TrigMode::SAFE;

  HeatControl heat_ctrl_;

  bool first_loading_ = true;

  bool soft_start_finish_ = false;

  float dt_ = 0.0f;

  LibXR::MillisecondTimestamp now_ = 0;

  LibXR::MicrosecondTimestamp last_wakeup_;

  LibXR::MillisecondTimestamp last_change_angle_time_ = 0;

  LibXR::MillisecondTimestamp start_loading_time_ = 0;

  RMMotor* motor_fric_front_left_;
  RMMotor* motor_fric_front_right_;
  RMMotor* motor_fric_back_left_;
  RMMotor* motor_fric_back_right_;
  RMMotor* motor_trig_;

  LibXR::ErrorCode motor_state_;

  float trig_setpoint_angle_ = 0.0f;
  float trig_setpoint_speed_ = 0.0f;

  float trig_zero_angle_ = 0.0f;
  float trig_angle_ = 0.0f;
  float trig_output_ = 0.0f;

  float fric_target_speed_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  LibXR::PID<float> trig_angle_pid_;
  LibXR::PID<float> trig_speed_pid_;

  LibXR::PID<float> fric_speed_pid_[4] = {
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param())};

  float current_back_left_ = 0.0f;

  bool fire_flag_ = false;    // 发射命令标志位
  uint8_t fired_ = 0;         // 已发射弹丸
  bool enable_fire_ = false;  // 拨弹盘旋转命令发出标志位
  bool mark_launch_ = false;  // 拨弹发射完成标志位

  LibXR::MillisecondTimestamp start_fire_time_ = 0;
  LibXR::MillisecondTimestamp finish_fire_time_ = 0;
  uint32_t real_launch_delay_ = 0.0f;

  bool last_fire_notify_ = false;
  bool press_continue_ = false;
  LibXR::MillisecondTimestamp fire_press_time_ = 0;

  uint16_t delay_time_ = 0;

  LauncherEvent launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;

  Motor::Feedback param_motor_fric_front_left_;
  Motor::Feedback param_motor_fric_front_right_;
  Motor::Feedback param_motor_fric_back_left_;
  Motor::Feedback param_motor_fric_back_right_;
  Motor::Feedback param_trig_;

  Motor::MotorCmd cmd_fric_front_left_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0};
  Motor::MotorCmd cmd_fric_front_right_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0};
  Motor::MotorCmd cmd_fric_back_left_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0};
  Motor::MotorCmd cmd_fric_back_right_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0};
  Motor::MotorCmd cmd_trig_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 19.2032f,
                      .velocity = 0};

  /*----------工具函数--------------------------------*/

  /**
   * @brief 更新拨弹盘模式
   */
  void UpdateTrigMode() {
    LibXR::MillisecondTimestamp now_time = LibXR::Timebase::GetMilliseconds();

    if (launcher_event_ != LauncherEvent::SET_FRICMODE_RELAX) {
      if (launcher_cmd_.isfire && !last_fire_notify_) {
        fire_press_time_ = now_time;
        press_continue_ = false;
        trig_mode_ = TrigMode::SINGLE;
      } else if (launcher_cmd_.isfire && last_fire_notify_) {
        if (!press_continue_ && (now_time - fire_press_time_ > 200)) {
          press_continue_ = true;
        }
        if (press_continue_) {
          trig_mode_ = TrigMode::CONTINUE;
        }
      } else {
        trig_mode_ = TrigMode::SAFE;
        press_continue_ = false;
      }
    } else {
      trig_mode_ = TrigMode::RELAX;
    }

    last_fire_notify_ = launcher_cmd_.isfire;
  }

  /**
   * @brief 根据模式设置摩擦轮目标转速
   */
  void UpdateFricTarget() {
    switch (launcher_event_) {
      case LauncherEvent::SET_FRICMODE_RELAX:
      case LauncherEvent::SET_FRICMODE_SAFE:
        fric_target_speed_[0] = 0;
        fric_target_speed_[1] = 0;
        fric_target_speed_[2] = 0;
        fric_target_speed_[3] = 0;
        soft_start_finish_ = false;
        Reset();
        break;
      case LauncherEvent::SET_FRICMODE_READY:
        fric_target_speed_[0] = param_.fric2_setpoint_speed;
        fric_target_speed_[1] = param_.fric2_setpoint_speed;
        fric_target_speed_[2] = param_.fric1_setpoint_speed;
        fric_target_speed_[3] = param_.fric1_setpoint_speed;
        break;
      default:
        break;
    }
  }

  /**
   * @brief 首次发弹标定控制
   */
  void FirstLoadingControl() {
    if (trig_mode_ == TrigMode::SINGLE) {
      fire_flag_ = true;
    }
    if (fire_flag_) {
      if (start_loading_time_ == 0) {
        start_loading_time_ = LibXR::Timebase::GetMilliseconds();
      }

      trig_setpoint_angle_ -= TRIG_LOADING_ANGLE_STEP;
      last_change_angle_time_ = LibXR::Timebase::GetMilliseconds();

      delay_time_++;
    }

    if (soft_start_finish_) {
      if (std::abs(param_motor_fric_back_left_.torque) > 0.04) {  // 发弹检测
        trig_zero_angle_ = trig_angle_;  // 获取电机当前位置
        trig_setpoint_angle_ = trig_angle_ - TRIG_ZERO_ANGLE_OFFSET;  // 偏移量

        fire_flag_ = false;
        first_loading_ = false;
        fired_++;

        mark_launch_ = true;
      }
    }
  }

  /**
   * @brief 常规发弹逻辑
   */
  void NormalFireControl() {
    if (trig_mode_ == TrigMode::SINGLE) {
      if (!enable_fire_ && mark_launch_) {
        if (heat_ctrl_.available_shot) {
          trig_setpoint_angle_ -= static_cast<float>(M_2PI) /
                                  static_cast<float>(param_.num_trig_tooth);

          enable_fire_ = true;
          mark_launch_ = false;
          start_fire_time_ = LibXR::Timebase::GetMilliseconds();

          trig_mode_ = TrigMode::SAFE;
        }
      }
    }
    now_ = LibXR::Timebase::GetMilliseconds();

    // 添加发射超时检测（超过1000毫秒未检测到发弹则重置状态）
    if (start_fire_time_ > 0 && (now_ - start_fire_time_ > 1000) &&
        !mark_launch_) {
      fire_flag_ = false;
      enable_fire_ = false;
      mark_launch_ = true;
      start_fire_time_ = now_;
    }

    if (!mark_launch_) {  // 发弹状态检测
      if (std::abs(param_motor_fric_back_left_.torque) > 0.04) {
        fire_flag_ = false;

        fired_++;

        mark_launch_ = true;
        enable_fire_ = false;
        finish_fire_time_ = LibXR::Timebase::GetMilliseconds();
      }
    }
  }

  /**
   * @brief 摩擦轮PID控制输出
   */
  void FricPidControl() {
    cmd_fric_front_left_.velocity = fric_speed_pid_[0].Calculate(
        fric_target_speed_[0], param_motor_fric_front_left_.velocity, dt_);
    cmd_fric_front_right_.velocity = fric_speed_pid_[1].Calculate(
        fric_target_speed_[1], param_motor_fric_front_right_.velocity, dt_);
    cmd_fric_back_left_.velocity = fric_speed_pid_[2].Calculate(
        fric_target_speed_[2], param_motor_fric_back_left_.velocity, dt_);
    cmd_fric_back_right_.velocity = fric_speed_pid_[3].Calculate(
        fric_target_speed_[3], param_motor_fric_back_right_.velocity, dt_);

    motor_fric_front_left_->Control(cmd_fric_front_left_);
    motor_fric_front_right_->Control(cmd_fric_front_right_);
    motor_fric_back_left_->Control(cmd_fric_back_left_);
    motor_fric_back_right_->Control(cmd_fric_back_right_);
  }

  /**
   * @brief 拨弹PID控制输出
   */
  void TrigPidControl() {
    trig_setpoint_speed_ =
        trig_angle_pid_.Calculate(trig_setpoint_angle_, trig_angle_, dt_);

    trig_output_ = trig_speed_pid_.Calculate(trig_setpoint_speed_,
                                             param_trig_.velocity, dt_);
    switch (trig_mode_) {
      case TrigMode::RELAX:
        cmd_trig_.velocity = 0;
        break;
      case TrigMode::SAFE:
      case TrigMode::SINGLE:
      case TrigMode::CONTINUE:
        cmd_trig_.velocity = trig_output_;
        break;
      default:
        break;
    }
    motor_trig_->Control(cmd_trig_);
  }

  /**
   * @brief 摩擦轮软启动控制
   * @details 在摩擦轮启动初期限制 PID 输出，防止电流冲击；
   *          当实际转速达到设定值后解除输出限制。
   */
  void SoftStart() {
    if (!soft_start_finish_) {
      for (LibXR::PID<float>& i : fric_speed_pid_) {
        i.SetOutLimit(0.05f);
      }

      if (motor_fric_back_left_->GetFeedback().velocity >
          param_.fric1_setpoint_speed) {
        soft_start_finish_ = true;
      }
    } else {
      for (LibXR::PID<float>& i : fric_speed_pid_) {
        i.SetOutLimit(1.0f);
      }
    }
  }

  /**
   * @brief 热量限制计算
   */
  void HeatLimit() {
    heat_ctrl_.heat_limit = ref_data_.rs.shooter_heat_limit;
    heat_ctrl_.heat_increase = 100.0f;
    heat_ctrl_.cooling_rate = ref_data_.rs.shooter_cooling_value;
    if (fired_ >= 1) {
      heat_ctrl_.heat += heat_ctrl_.heat_increase;
      fired_ = 0;
    }
    heat_ctrl_.heat -=
        heat_ctrl_.cooling_rate / (1 / dt_);  // 每个控制周期的冷却恢复
    heat_ctrl_.heat = std::max(heat_ctrl_.heat, 0.0f);
    float available_float =
        (this->heat_ctrl_.heat_limit - this->heat_ctrl_.heat) /
        this->heat_ctrl_.heat_increase;
    heat_ctrl_.available_shot = static_cast<uint32_t>(available_float);
  }
};

#ifdef DEBUG
#define HERO_LAUNCHER_DEBUG_IMPL
#include "HeroLauncherDebug.inl"
#undef HERO_LAUNCHER_DEBUG_IMPL
#endif
