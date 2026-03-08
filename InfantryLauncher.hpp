#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#ifdef DEBUG
#include "DebugCore.hpp"
#include "ramfs.hpp"
#endif
#include "Motor.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "timebase.hpp"

namespace launcher::param {
constexpr float TRIG_STEP = static_cast<float>(M_2PI) / 10.0f;
constexpr float JAM_TORQUE = 0.1f;
constexpr float FRIC_DROP_RPM = 50.0f;
constexpr float JAM_TOGGLE_INTERVAL_SEC = 0.02f;
constexpr float LONG_PRESS_THRESHOLD_SEC = 0.5f;
constexpr float HEAT_TICK_SEC = 0.05f;
}  // namespace launcher::param

/**
 * @brief 步兵发射机构实现
 * @details 负责摩擦轮、拨弹盘控制与热量约束发射逻辑。
 *          作为 Launcher<InfantryLauncher> 的内部逻辑类，不拥有线程和事件注册。
 */
class InfantryLauncher {
 public:
  enum class LauncherState : uint8_t {
    RELAX,
    STOP,
    NORMAL,
    JAMMED,
  };

  enum class LauncherEvent : uint8_t {
    SET_FRICMODE_RELAX,
    SET_FRICMODE_SAFE,
    SET_FRICMODE_READY,
  };

  enum class TrigMode : uint8_t {
    RELAX,
    SAFE,
    SINGLE,
    CONTINUE,
    JAM,
  };

  struct RefereeData {
    float heat_limit;
    float heat_cooling;
  };

  struct LauncherParam {
    float fric1_setpoint_speed;
    float fric2_setpoint_speed;
    float trig_gear_ratio;
    uint8_t num_trig_tooth;
    float expect_trig_freq_;
  };

  struct HeatLimit {
    float single_heat;
    float launched_num;
    float current_heat;
    float heat_threshold;
    bool allow_fire;
  };

  /**
   * @brief 步兵发射器构造函数
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param motor_fric_front_left 左摩擦轮电机
   * @param motor_fric_front_right 右摩擦轮电机
   * @param motor_fric_back_left 后左摩擦轮电机（当前实现未使用）
   * @param motor_fric_back_right 后右摩擦轮电机（当前实现未使用）
   * @param motor_trig 拨弹电机
   * @param task_stack_depth 控制线程栈深度（由外壳使用）
   * @param pid_param_trig_angle 拨弹角度环参数
   * @param pid_param_trig_speed 拨弹速度环参数
   * @param pid_param_fric_0 摩擦轮0 PID参数
   * @param pid_param_fric_1 摩擦轮1 PID参数
   * @param pid_param_fric_2 预留参数（当前实现未使用）
   * @param pid_param_fric_3 预留参数（当前实现未使用）
   * @param launch_param 发射机构参数
   * @param cmd CMD模块指针
   */
  InfantryLauncher(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                   RMMotor* motor_fric_front_left,
                   RMMotor* motor_fric_front_right,
                   RMMotor* motor_fric_back_left,
                   RMMotor* motor_fric_back_right, RMMotor* motor_trig,
                   uint32_t task_stack_depth,
                   LibXR::PID<float>::Param pid_param_trig_angle,
                   LibXR::PID<float>::Param pid_param_trig_speed,
                   LibXR::PID<float>::Param pid_param_fric_0,
                   LibXR::PID<float>::Param pid_param_fric_1,
                   LibXR::PID<float>::Param pid_param_fric_2,
                   LibXR::PID<float>::Param pid_param_fric_3,
                   LauncherParam launch_param, CMD* cmd)
      : motor_fric_0_(motor_fric_front_left),
        motor_fric_1_(motor_fric_front_right),
        motor_trig_(motor_trig),
        pid_trig_angle_(pid_param_trig_angle),
        pid_trig_sp_(pid_param_trig_speed),
        pid_fric_0_(pid_param_fric_0),
        pid_fric_1_(pid_param_fric_1),
        param_(launch_param) {
    UNUSED(hw);
    UNUSED(app);
    UNUSED(task_stack_depth);
    UNUSED(pid_param_fric_2);
    UNUSED(pid_param_fric_3);
    UNUSED(motor_fric_back_left);
    UNUSED(motor_fric_back_right);
    UNUSED(cmd);

    last_online_time_ = LibXR::Timebase::GetMilliseconds();
    last_heat_time_ = LibXR::Timebase::GetMilliseconds();
  }

  /**
   * @brief 数据处理主入口
   * @details 更新周期时间、电机反馈、拨弹角度，并刷新发射器总状态。
   */
  void Update() {
    auto now = LibXR::Timebase::GetMilliseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    referee_data_.heat_limit = 260.0f;
    referee_data_.heat_cooling = 20.0f;
    heat_limit_.single_heat = 10.0f;
    heat_limit_.heat_threshold = 2.0f;

    motor_fric_0_->Update();
    motor_fric_1_->Update();
    motor_trig_->Update();

    param_fric_0_ = motor_fric_0_->GetFeedback();
    param_fric_1_ = motor_fric_1_->GetFeedback();
    param_trig_ = motor_trig_->GetFeedback();

    float current_motor_angle = param_trig_.position;
    float delta_trig_angle = LibXR::CycleValue<float>(current_motor_angle) -
                             LibXR::CycleValue<float>(last_motor_angle_);
    trig_angle_ += delta_trig_angle / param_.trig_gear_ratio;
    last_motor_angle_ = current_motor_angle;

    UpdateLauncherState();
  }

  /**
   * @brief 状态机主入口
   * @details
   * 根据当前状态、命令输入和热量限制计算目标拨弹角度和摩擦轮转速，并处理卡弹逻辑。
   */
  void Solve() {
    UpdateHeatControl();
    RunStateMachine();
    UpdateShotLatency();
    PublishTopics();
  }

  /**
   * @brief 控制输出
   * @details 计算拨盘与摩擦轮控制量并下发到电机，包含电机状态检查和错误恢复。
   */
  void Control() {
    float out_trig = 0.0f;
    float out_fric_0 = 0.0f;
    float out_fric_1 = 0.0f;
    Motor::Feedback trig_fb{};
    Motor::Feedback fric_0_fb{};
    Motor::Feedback fric_1_fb{};
    bool relax = false;

    SetFricTargetByEvent();

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_RELAX) {
      relax = true;
    } else {
      if (trig_mode_ != TrigMode::RELAX) {
        TrigControl(out_trig, target_trig_angle_, dt_);
      }
      FricControl(out_fric_0, out_fric_1, target_rpm_, dt_);
      trig_fb = param_trig_;
      fric_0_fb = param_fric_0_;
      fric_1_fb = param_fric_1_;
    }

    if (relax) {
      motor_trig_->Relax();
      motor_fric_0_->Relax();
      motor_fric_1_->Relax();
      return;
    }

    auto cmd_trig = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                    .reduction_ratio = 36.0f,
                                    .velocity = out_trig};
    auto cmd_fric_0 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 19.0f,
                                      .velocity = out_fric_0};
    auto cmd_fric_1 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 19.0f,
                                      .velocity = out_fric_1};

    auto motor_control = [&](Motor* motor, const Motor::Feedback& fb,
                             const Motor::MotorCmd& cmd) {
      if (fb.state == 0) {
        motor->Enable();
      } else if (fb.state != 0 && fb.state != 1) {
        motor->ClearError();
      } else {
        motor->Control(cmd);
      }
    };

    motor_control(motor_trig_, trig_fb, cmd_trig);
    motor_control(motor_fric_0_, fric_0_fb, cmd_fric_0);
    motor_control(motor_fric_1_, fric_1_fb, cmd_fric_1);
  }

  /**
   * @brief 设置摩擦轮模式事件
   * @param mode 事件ID，对应 LauncherEvent
   * @details 切换模式后同步复位相关PID，避免模式切换瞬态冲击。
   */
  void SetMode(uint32_t mode) {
    launcher_event_ = static_cast<LauncherEvent>(mode);
    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();
  }

  /**
   * @brief 失控处理
   * @details 复位状态机与PID并关闭输出，确保发射机构进入安全状态。
   */
  void LostCtrl() {
    launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
    launcher_state_ = LauncherState::RELAX;
    trig_mode_ = TrigMode::RELAX;

    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();

    target_trig_angle_ = trig_angle_;
    shoot_active_ = false;
    shot_start_time_ = 0;
    press_continue_ = false;
    launcher_cmd_.isfire = false;

    motor_trig_->Disable();
    motor_fric_0_->Relax();
    motor_fric_1_->Relax();
  }

  /**
   * @brief 监控回调
   */
  void OnMonitor() {}

  /**
   * @brief 调试命令入口
   */
#ifdef DEBUG
  int DebugCommand(int argc, char** argv);
#endif

  /* 外壳可直接写入的命令数据 */
  CMD::LauncherCMD launcher_cmd_{};  // NOLINT

 private:
  RMMotor* motor_fric_0_;
  RMMotor* motor_fric_1_;
  RMMotor* motor_trig_;

  Motor::Feedback param_fric_0_{};
  Motor::Feedback param_fric_1_{};
  Motor::Feedback param_trig_{};

  LibXR::PID<float> pid_trig_angle_;
  LibXR::PID<float> pid_trig_sp_;
  LibXR::PID<float> pid_fric_0_;
  LibXR::PID<float> pid_fric_1_;

  LauncherParam param_;

  float dt_ = 0.0f;
  float target_rpm_ = 0.0f;
  float trig_freq_ = 0.0f;

  float trig_angle_ = 0.0f;
  float target_trig_angle_ = 0.0f;
  float last_motor_angle_ = 0.0f;

  float number_ = 0.0f;
  float shoot_dt_ = 0.0f;

  bool last_fire_notify_ = false;
  bool press_continue_ = false;
  bool is_reverse_ = false;
  bool shoot_active_ = false;

  float jam_keep_time_s_ = 0.0f;

  LibXR::MillisecondTimestamp fire_press_time_ = 0;
  LibXR::MillisecondTimestamp last_trig_time_ = 0;
  LibXR::MillisecondTimestamp last_jam_time_ = 0;
  LibXR::MillisecondTimestamp last_heat_time_ = 0;
  LibXR::MillisecondTimestamp last_online_time_ = 0;
  LibXR::MillisecondTimestamp shoot_time_ = 0;
  LibXR::MillisecondTimestamp receive_fire_time_ = 0;
  LibXR::MillisecondTimestamp shot_start_time_ = 0;

  LibXR::Topic shoot_waiting_ = LibXR::Topic::CreateTopic<float>("shoot_dt");
  LibXR::Topic shoot_number_ = LibXR::Topic::CreateTopic<float>("shoot_number");
  LibXR::Topic shoot_freq_ = LibXR::Topic::CreateTopic<float>("trig_freq");

  LauncherEvent launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
  LauncherState launcher_state_ = LauncherState::RELAX;
  TrigMode trig_mode_ = TrigMode::RELAX;
  TrigMode last_trig_mode_ = TrigMode::RELAX;

  RefereeData referee_data_{.heat_limit = 0.0f, .heat_cooling = 0.0f};
  HeatLimit heat_limit_{
      .single_heat = 0.0f,
      .launched_num = 0.0f,
      .current_heat = 0.0f,
      .heat_threshold = 0.0f,
      .allow_fire = true,
  };

  /*-----------------工具函数---------------------------------------------------*/

  /**
   * @brief 更新发射器总状态
   * @details 基于卡弹判据、摩擦轮模式与热量许可，计算
   * RELAX/STOP/NORMAL/JAMMED。
   */
  void UpdateLauncherState() {
    if (fabsf(param_trig_.torque) > launcher::param::JAM_TORQUE) {
      launcher_state_ = LauncherState::JAMMED;
      return;
    }

    if (launcher_event_ != LauncherEvent::SET_FRICMODE_READY) {
      launcher_state_ = LauncherState::RELAX;
      return;
    }

    if (!heat_limit_.allow_fire) {
      launcher_state_ = LauncherState::STOP;
      return;
    }

    launcher_state_ =
        launcher_cmd_.isfire ? LauncherState::NORMAL : LauncherState::STOP;
  }

  /**
   * @brief 执行拨弹状态机
   * @details 依次更新拨盘模式、拨盘目标角度和发射判定，并刷新按键边沿状态。
   */
  void RunStateMachine() {
    auto now = LibXR::Timebase::GetMilliseconds();
    UpdateTriggerMode(now);
    UpdateTriggerSetpoint(now);
    UpdateShotJudge(now);
    last_fire_notify_ = launcher_cmd_.isfire;
  }

  /**
   * @brief 根据发射器状态切换拨盘模式
   * @param now 当前时间戳
   * @details 处理单发/连发/卡弹模式切换，其中连发由按键长按时长触发。
   */
  void UpdateTriggerMode(LibXR::MillisecondTimestamp now) {
    switch (launcher_state_) {
      case LauncherState::RELAX:
        trig_mode_ = TrigMode::RELAX;
        press_continue_ = false;
        break;

      case LauncherState::STOP:
        trig_mode_ = TrigMode::SAFE;
        press_continue_ = false;
        break;

      case LauncherState::NORMAL:
        if (!last_fire_notify_) {
          fire_press_time_ = now;
          press_continue_ = false;
          trig_mode_ = TrigMode::SINGLE;
        } else {
          if (!press_continue_ &&
              (now - fire_press_time_).ToSecondf() >
                  launcher::param::LONG_PRESS_THRESHOLD_SEC) {
            press_continue_ = true;
          }
          trig_mode_ = press_continue_ ? TrigMode::CONTINUE : TrigMode::SINGLE;
        }
        break;

      case LauncherState::JAMMED:
        trig_mode_ = TrigMode::JAM;
        break;
    }
  }

  /**
   * @brief 更新拨盘目标角度
   * @param now 当前时间戳
   * @details 根据 TrigMode 生成目标角度；卡弹模式下周期切换正反向退弹角度。
   */
  void UpdateTriggerSetpoint(LibXR::MillisecondTimestamp now) {
    switch (trig_mode_) {
      case TrigMode::RELAX:
      case TrigMode::SAFE:
        target_trig_angle_ = trig_angle_;
        shoot_active_ = false;
        shot_start_time_ = 0;
        break;

      case TrigMode::SINGLE:
        if (last_trig_mode_ == TrigMode::SAFE ||
            last_trig_mode_ == TrigMode::RELAX ||
            last_trig_mode_ == TrigMode::JAM) {
          target_trig_angle_ = trig_angle_ + launcher::param::TRIG_STEP;
          shoot_active_ = true;
          shot_start_time_ = now;
        }
        break;

      case TrigMode::CONTINUE: {
        if (!shoot_active_) {
          float trig_freq = std::max(trig_freq_, 1e-3f);
          float interval_s = 1.0f / trig_freq;
          float since_last = (now - last_trig_time_).ToSecondf();
          if (since_last >= interval_s) {
            target_trig_angle_ = trig_angle_ + launcher::param::TRIG_STEP;
            last_trig_time_ = now;
            shoot_active_ = true;
            shot_start_time_ = now;
          }
        }
      } break;

      case TrigMode::JAM: {
        shoot_active_ = false;
        shot_start_time_ = 0;
        jam_keep_time_s_ = (now - last_jam_time_).ToSecondf();
        if (jam_keep_time_s_ >= launcher::param::JAM_TOGGLE_INTERVAL_SEC) {
          if (last_trig_mode_ != TrigMode::JAM) {
            is_reverse_ = true;
          }
          target_trig_angle_ =
              trig_angle_ + (is_reverse_ ? -2.0f * launcher::param::TRIG_STEP
                                         : launcher::param::TRIG_STEP);
          is_reverse_ = !is_reverse_;
          last_jam_time_ = now;
        }
      } break;
    }

    last_trig_mode_ = trig_mode_;
  }

  /**
   * @brief 发射成功判定
   * @param now 当前时间戳
   * @details 通过摩擦轮转速跌落判定出弹，并更新热量计数和累计发射数。
   */
  void UpdateShotJudge(LibXR::MillisecondTimestamp now) {
    if (!shoot_active_) {
      return;
    }

    bool success =
        (fabsf(param_fric_0_.velocity) <
         (param_.fric1_setpoint_speed - launcher::param::FRIC_DROP_RPM)) &&
        (fabsf(param_fric_1_.velocity) <
         (param_.fric1_setpoint_speed - launcher::param::FRIC_DROP_RPM));

    if (success) {
      shoot_time_ = now;
      heat_limit_.launched_num += 1.0f;
      shoot_active_ = false;
      shot_start_time_ = 0;
      number_ += 1.0f;
      return;
    }

    if (shot_start_time_ != 0 && (now - shot_start_time_).ToSecondf() > 0.2f) {
      shoot_active_ = false;
      shot_start_time_ = 0;
    }
  }

  /**
   * @brief 更新发射时延
   * @details 记录发射命令边沿时间与出弹时间差，输出 shoot_dt。
   */
  void UpdateShotLatency() {
    auto now = LibXR::Timebase::GetMilliseconds();
    if (!last_fire_notify_ && launcher_cmd_.isfire) {
      receive_fire_time_ = now;
    }

    if (receive_fire_time_ <= shoot_time_) {
      shoot_dt_ = (shoot_time_ - receive_fire_time_).ToSecondf();
    }
  }

  /**
   * @brief 根据模式设置摩擦轮目标转速
   * @details RELAX/SAFE 下目标转速为0，READY 下使用配置转速。
   */
  void SetFricTargetByEvent() {
    switch (launcher_event_) {
      case LauncherEvent::SET_FRICMODE_RELAX:
      case LauncherEvent::SET_FRICMODE_SAFE:
        target_rpm_ = 0.0f;
        break;
      case LauncherEvent::SET_FRICMODE_READY:
        target_rpm_ = param_.fric1_setpoint_speed;
        break;
      default:
        break;
    }
  }

  /**
   * @brief 热量管理与弹频调度
   * @details 周期更新当前热量，计算是否允许发射，并依据剩余热量调整目标弹频。
   */
  void UpdateHeatControl() {
    auto now = LibXR::Timebase::GetMilliseconds();
    float delta_time = (now - last_heat_time_).ToSecondf();
    if (delta_time < launcher::param::HEAT_TICK_SEC) {
      return;
    }
    last_heat_time_ = now;

    heat_limit_.current_heat +=
        heat_limit_.single_heat * heat_limit_.launched_num;
    heat_limit_.launched_num = 0.0f;

    if (heat_limit_.current_heat <
        static_cast<float>(referee_data_.heat_cooling / 10.0f)) {
      heat_limit_.current_heat = 0.0f;
    } else {
      heat_limit_.current_heat -=
          static_cast<float>(referee_data_.heat_cooling / 10.0f);
    }

    float residuary_heat = referee_data_.heat_limit - heat_limit_.current_heat;
    heat_limit_.allow_fire = residuary_heat > heat_limit_.single_heat;

    if (!heat_limit_.allow_fire) {
      trig_freq_ = referee_data_.heat_cooling / heat_limit_.single_heat;
      return;
    }

    if (residuary_heat <=
        heat_limit_.single_heat * heat_limit_.heat_threshold) {
      float safe_freq = referee_data_.heat_cooling / heat_limit_.single_heat;
      float ratio = residuary_heat /
                    (heat_limit_.single_heat * heat_limit_.heat_threshold);
      trig_freq_ = ratio * (param_.expect_trig_freq_ - safe_freq) + safe_freq;
      return;
    }

    trig_freq_ = param_.expect_trig_freq_;
  }

  /**
   * @brief 发布调试与统计话题
   * @details 发布发射等待时间、累计发射数和当前弹频。
   */
  void PublishTopics() {
    shoot_waiting_.Publish(shoot_dt_);
    shoot_number_.Publish(number_);
    shoot_freq_.Publish(trig_freq_);
  }

  /**
   * @brief 拨盘控制解算
   * @param out_trig 拨盘控制输出
   * @param target_trig_angle 拨盘目标角度
   * @param dt 控制周期
   * @details 角度环生成参考速度，速度环生成最终控制输出，并进行速度限幅。
   */
  void TrigControl(float& out_trig, float target_trig_angle, float dt) {
    float plate_omega_ref = pid_trig_angle_.Calculate(
        target_trig_angle, trig_angle_,
        param_trig_.omega / param_.trig_gear_ratio, dt);
    float omega_limit =
        static_cast<float>(1.5f * M_2PI * trig_freq_ / param_.num_trig_tooth);
    float motor_omega_ref =
        std::clamp(plate_omega_ref, -omega_limit, omega_limit);
    out_trig = pid_trig_sp_.Calculate(
        motor_omega_ref, param_trig_.omega / param_.trig_gear_ratio, dt);
  }

  /**
   * @brief 摩擦轮控制解算
   * @param out_fric_0 摩擦轮0控制输出
   * @param out_fric_1 摩擦轮1控制输出
   * @param target_rpm 摩擦轮目标转速
   * @param dt 控制周期
   * @details 速度环计算摩擦轮输出，SAFE 模式下对输出限幅做缓停处理。
   */
  void FricControl(float& out_fric_0, float& out_fric_1, float target_rpm,
                   float dt) {
    out_fric_0 = pid_fric_0_.Calculate(target_rpm, param_fric_0_.velocity, dt);
    out_fric_1 = pid_fric_1_.Calculate(target_rpm, param_fric_1_.velocity, dt);

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_SAFE) {
      pid_fric_0_.SetOutLimit(1.5f);
      pid_fric_1_.SetOutLimit(1.5f);
    }
  }
};

#ifdef DEBUG
#define INFANTRY_LAUNCHER_DEBUG_IMPL
#include "InfantryLauncherDebug.inl"
#undef INFANTRY_LAUNCHER_DEBUG_IMPL
#endif
