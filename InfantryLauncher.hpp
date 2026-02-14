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
#include "event.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
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

  enum class TRIGMODE : uint8_t {
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
   * @param task_stack_depth 控制线程栈深度
   * @param pid_param_trig_angle 拨弹角度环参数
   * @param pid_param_trig_speed 拨弹速度环参数
   * @param pid_param_fric_0 摩擦轮0 PID参数
   * @param pid_param_fric_1 摩擦轮1 PID参数
   * @param pid_param_fric_2 预留参数（当前实现未使用）
   * @param pid_param_fric_3 预留参数（当前实现未使用）
   * @param launch_param 发射机构参数
   * @param cmd CMD模块指针
   * @details 完成线程创建、失控事件注册和调试命令文件注册。
   */
  InfantryLauncher(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
                   RMMotor *motor_fric_front_left,
                   RMMotor *motor_fric_front_right,
                   RMMotor *motor_fric_back_left,
                   RMMotor *motor_fric_back_right,
                   RMMotor *motor_trig,
                   uint32_t task_stack_depth,
                   LibXR::PID<float>::Param pid_param_trig_angle,
                   LibXR::PID<float>::Param pid_param_trig_speed,
                   LibXR::PID<float>::Param pid_param_fric_0,
                   LibXR::PID<float>::Param pid_param_fric_1,
                   LibXR::PID<float>::Param pid_param_fric_2,
                   LibXR::PID<float>::Param pid_param_fric_3,
                   LauncherParam launch_param,
                   CMD *cmd)
      : motor_fric_0_(motor_fric_front_left),
        motor_fric_1_(motor_fric_front_right),
        motor_trig_(motor_trig),
        pid_trig_angle_(pid_param_trig_angle),
        pid_trig_sp_(pid_param_trig_speed),
        pid_fric_0_(pid_param_fric_0),
        pid_fric_1_(pid_param_fric_1),
        param_(launch_param),
        cmd_(cmd),
        cmd_file_(LibXR::RamFS::CreateFile("launcher", CommandFunc, this)) {
    UNUSED(app);
    UNUSED(task_stack_depth);
    UNUSED(pid_param_fric_2);
    UNUSED(pid_param_fric_3);
    UNUSED(motor_fric_back_left);
    UNUSED(motor_fric_back_right);

    thread_.Create(this, ThreadFunction, "LauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::HIGH);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher* launcher, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          launcher->LostCtrl();
        },
        this);

    auto callback = LibXR::Callback<uint32_t>::Create([](bool in_isr, InfantryLauncher *launcher, uint32_t event_id) {
      UNUSED(in_isr);
      launcher->SetMode(static_cast<LauncherEvent>(event_id));
    }, this);
    cmd_->GetEvent().Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX), callback);
    cmd_->GetEvent().Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_SAFE), callback);
    cmd_->GetEvent().Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_READY), callback);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);

    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
  }

  /**
   * @brief 发射器主线程函数
   * @param launcher InfantryLauncher对象指针
   * @details 周期执行数据更新、热量管理、状态机计算、控制输出与调试话题发布。
   */
  static void ThreadFunction(InfantryLauncher *launcher) {
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> launcher_cmd_sub(
        "launcher_cmd");
    launcher_cmd_sub.StartWaiting();

    launcher->last_online_time_ = LibXR::Timebase::GetMilliseconds();
    launcher->last_heat_time_ = LibXR::Timebase::GetMilliseconds();

    while (true) {
      auto last_time = LibXR::Timebase::GetMilliseconds();

      if (launcher_cmd_sub.Available()) {
        launcher->launcher_cmd_ = launcher_cmd_sub.GetData();
        launcher_cmd_sub.StartWaiting();
      }

      launcher->mutex_.Lock();
      launcher->Update();
      launcher->Solve();
      launcher->Control();
      launcher->PublishTopics();
      launcher->mutex_.Unlock();

      LibXR::Thread::SleepUntil(last_time, 2);
    }
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
    * @details 根据当前状态、命令输入和热量限制计算目标拨弹角度和摩擦轮转速，并处理卡弹逻辑。
    */
  void Solve(){
    UpdateHeatControl();
    RunStateMachine();
    UpdateShotLatency();
  }

  /**
   * @brief 控制输出
   * @details 计算拨盘与摩擦轮控制量并下发到电机，包含电机状态检查和错误恢复。
   */
  void Control() {
    SetFricTargetByEvent();

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_RELAX) {
      motor_trig_->Relax();
      motor_fric_0_->Relax();
      motor_fric_1_->Relax();
      return;
    }

    float out_trig = 0.0f;
    float out_fric_0 = 0.0f;
    float out_fric_1 = 0.0f;

    if (trig_mode_ != TRIGMODE::RELAX) {
      TrigControl(out_trig, target_trig_angle_, dt_);
    }
    FricControl(out_fric_0, out_fric_1, target_rpm_, dt_);

    auto cmd_trig = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                    .reduction_ratio = 36.0f,
                                    .velocity = out_trig};
    auto cmd_fric_0 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 19.0f,
                                      .velocity = out_fric_0};
    auto cmd_fric_1 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 19.0f,
                                      .velocity = out_fric_1};

    auto motor_control = [&](Motor *motor, const Motor::Feedback &fb,
                             const Motor::MotorCmd &cmd) {
      if (fb.state == 0) {
        motor->Enable();
      } else if (fb.state != 0 && fb.state != 1) {
        motor->ClearError();
      } else {
        motor->Control(cmd);
      }
    };

    motor_control(motor_trig_, param_trig_, cmd_trig);
    motor_control(motor_fric_0_, param_fric_0_, cmd_fric_0);
    motor_control(motor_fric_1_, param_fric_1_, cmd_fric_1);
  }

  /**
   * @brief 设置摩擦轮模式事件
   * @param mode 事件ID，对应 LauncherEvent
   * @details 切换模式后同步复位相关PID，避免模式切换瞬态冲击。
   */
  void SetMode(LauncherEvent mode) {
    mutex_.Lock();
    launcher_event_ = mode;
    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();
    mutex_.Unlock();
  }

  /**
   * @brief 监控回调
   * @details 当前模块无额外监控任务，保留空实现。
   */
  void OnMonitor() {}

  /**
   * @brief 调试命令入口
   * @param argc 命令参数个数
   * @param argv 命令参数数组
   * @return int 命令执行结果，0表示成功，负值表示失败
   * @details 支持 state/motor/heat/shot/full 视图以及 once/monitor 调试模式。
   */
  int DebugCommand(int argc, char **argv) {
    enum class DebugView : uint8_t {
      STATE,
      MOTOR,
      HEAT,
      SHOT,
      FULL,
    };

    struct DebugSnapshot {
      LauncherEvent launcher_event;
      LauncherState launcher_state;
      TRIGMODE trig_mode;

      bool is_fire_cmd;
      bool allow_fire;
      bool shoot_active;
      bool jam_reverse;
      bool long_press;

      float dt;
      float target_rpm;
      float trig_freq;
      float trig_angle;
      float trig_target_angle;
      float number;
      float shoot_dt;
      float jam_keep_time_s;

      float heat_now;
      float heat_limit;
      float heat_cooling;
      float heat_single;

      Motor::Feedback fric_0;
      Motor::Feedback fric_1;
      Motor::Feedback trig;
    };

    auto event_to_string = [](LauncherEvent event) {
      switch (event) {
        case LauncherEvent::SET_FRICMODE_RELAX:
          return "RELAX";
        case LauncherEvent::SET_FRICMODE_SAFE:
          return "SAFE";
        case LauncherEvent::SET_FRICMODE_READY:
          return "READY";
        default:
          return "UNKNOWN";
      }
    };

    auto state_to_string = [](LauncherState state) {
      switch (state) {
        case LauncherState::RELAX:
          return "RELAX";
        case LauncherState::STOP:
          return "STOP";
        case LauncherState::NORMAL:
          return "NORMAL";
        case LauncherState::JAMMED:
          return "JAMMED";
        default:
          return "UNKNOWN";
      }
    };

    auto trig_mode_to_string = [](TRIGMODE mode) {
      switch (mode) {
        case TRIGMODE::RELAX:
          return "RELAX";
        case TRIGMODE::SAFE:
          return "SAFE";
        case TRIGMODE::SINGLE:
          return "SINGLE";
        case TRIGMODE::CONTINUE:
          return "CONTINUE";
        case TRIGMODE::JAM:
          return "JAM";
        default:
          return "UNKNOWN";
      }
    };

    auto view_to_string = [](DebugView view) {
      switch (view) {
        case DebugView::STATE:
          return "state";
        case DebugView::MOTOR:
          return "motor";
        case DebugView::HEAT:
          return "heat";
        case DebugView::SHOT:
          return "shot";
        case DebugView::FULL:
          return "full";
        default:
          return "unknown";
      }
    };

    auto parse_view = [](const char *arg, DebugView *view) {
      if (strcmp(arg, "state") == 0) {
        *view = DebugView::STATE;
        return true;
      }
      if (strcmp(arg, "motor") == 0) {
        *view = DebugView::MOTOR;
        return true;
      }
      if (strcmp(arg, "heat") == 0) {
        *view = DebugView::HEAT;
        return true;
      }
      if (strcmp(arg, "shot") == 0) {
        *view = DebugView::SHOT;
        return true;
      }
      if (strcmp(arg, "full") == 0) {
        *view = DebugView::FULL;
        return true;
      }
      return false;
    };

    auto capture_snapshot = [this]() {
      DebugSnapshot s{};
      mutex_.Lock();
      s.launcher_event = launcher_event_;
      s.launcher_state = launcher_state_;
      s.trig_mode = trig_mode_;

      s.is_fire_cmd = launcher_cmd_.isfire;
      s.allow_fire = heat_limit_.allow_fire;
      s.shoot_active = shoot_active_;
      s.jam_reverse = is_reverse_;
      s.long_press = press_continue_;

      s.dt = dt_;
      s.target_rpm = target_rpm_;
      s.trig_freq = trig_freq_;
      s.trig_angle = trig_angle_;
      s.trig_target_angle = target_trig_angle_;
      s.number = number_;
      s.shoot_dt = shoot_dt_;
      s.jam_keep_time_s = jam_keep_time_s_;

      s.heat_now = heat_limit_.current_heat;
      s.heat_limit = referee_data_.heat_limit;
      s.heat_cooling = referee_data_.heat_cooling;
      s.heat_single = heat_limit_.single_heat;

      s.fric_0 = param_fric_0_;
      s.fric_1 = param_fric_1_;
      s.trig = param_trig_;
      mutex_.Unlock();
      return s;
    };

    auto print_state = [&](const DebugSnapshot &s) {
      float hz = (s.dt > 1e-6f) ? (1.0f / s.dt) : 0.0f;
      LibXR::STDIO::Printf(
          "  state: event=%s(%u), launcher=%s, trig=%s, dt=%.4f s (%.1f Hz)\r\n",
          event_to_string(s.launcher_event),
          static_cast<unsigned>(s.launcher_event),
          state_to_string(s.launcher_state),
          trig_mode_to_string(s.trig_mode), s.dt, hz);
      LibXR::STDIO::Printf(
          "  cmd: fire=%d, allow_fire=%d, long_press=%d, shoot_active=%d\r\n",
          s.is_fire_cmd ? 1 : 0, s.allow_fire ? 1 : 0, s.long_press ? 1 : 0,
          s.shoot_active ? 1 : 0);
    };

    auto print_motor = [&](const DebugSnapshot &s) {
      LibXR::STDIO::Printf(
          "  motor: target_rpm=%.1f, trig_target=%.3f rad, trig_now=%.3f rad\r\n",
          s.target_rpm, s.trig_target_angle, s.trig_angle);
      LibXR::STDIO::Printf(
          "    fric0: rpm=%.1f, omega=%.3f, torque=%.3f, state=%u\r\n",
          s.fric_0.velocity, s.fric_0.omega, s.fric_0.torque,
          static_cast<unsigned>(s.fric_0.state));
      LibXR::STDIO::Printf(
          "    fric1: rpm=%.1f, omega=%.3f, torque=%.3f, state=%u\r\n",
          s.fric_1.velocity, s.fric_1.omega, s.fric_1.torque,
          static_cast<unsigned>(s.fric_1.state));
      LibXR::STDIO::Printf(
          "    trig : rpm=%.1f, omega=%.3f, torque=%.3f, state=%u\r\n",
          s.trig.velocity, s.trig.omega, s.trig.torque,
          static_cast<unsigned>(s.trig.state));
    };

    auto print_heat = [&](const DebugSnapshot &s) {
      LibXR::STDIO::Printf(
          "  heat: now=%.1f, limit=%.1f, cooling=%.1f, single=%.1f, trig_freq=%.2f Hz\r\n",
          s.heat_now, s.heat_limit, s.heat_cooling, s.heat_single, s.trig_freq);
    };

    auto print_shot = [&](const DebugSnapshot &s) {
      LibXR::STDIO::Printf(
          "  shot: count=%.0f, latency=%.3f s, jam_keep=%.3f s, jam_reverse=%d\r\n",
          s.number, s.shoot_dt, s.jam_keep_time_s, s.jam_reverse ? 1 : 0);
    };

    auto print_once = [&](DebugView view) {
      DebugSnapshot s = capture_snapshot();
      LibXR::STDIO::Printf("[%lu ms] launcher %s\r\n", LibXR::Thread::GetTime(),
                           view_to_string(view));
      switch (view) {
        case DebugView::STATE:
          print_state(s);
          break;
        case DebugView::MOTOR:
          print_motor(s);
          break;
        case DebugView::HEAT:
          print_heat(s);
          break;
        case DebugView::SHOT:
          print_shot(s);
          break;
        case DebugView::FULL:
          print_state(s);
          print_motor(s);
          print_heat(s);
          print_shot(s);
          break;
      }
    };

    if (argc == 1) {
      LibXR::STDIO::Printf("Usage:\r\n");
      LibXR::STDIO::Printf("  monitor\r\n");
      LibXR::STDIO::Printf(
          "  monitor <time_ms> [interval_ms] [state|motor|heat|shot|full]\r\n");
      LibXR::STDIO::Printf("  once [state|motor|heat|shot|full]\r\n");
      LibXR::STDIO::Printf("  state | motor | heat | shot | full\r\n");
      return 0;
    }

    if (strcmp(argv[1], "monitor") == 0) {
      if (argc == 2) {
        print_once(DebugView::FULL);
        return 0;
      }

      if (argc > 5) {
        LibXR::STDIO::Printf("Error: Too many arguments for monitor.\r\n");
        return -1;
      }

      int time_ms = atoi(argv[2]);
      int interval_ms = 1000;
      DebugView view = DebugView::FULL;
      bool third_is_view = false;

      if (argc >= 4) {
        DebugView parsed_view = DebugView::FULL;
        if (parse_view(argv[3], &parsed_view)) {
          view = parsed_view;
          third_is_view = true;
        } else {
          interval_ms = atoi(argv[3]);
        }
      }

      if (argc == 5) {
        if (third_is_view) {
          LibXR::STDIO::Printf(
              "Error: Invalid monitor args. Use monitor <time_ms> [interval_ms] [view].\r\n");
          return -1;
        }
        if (!parse_view(argv[4], &view)) {
          LibXR::STDIO::Printf("Error: Unknown view '%s'.\r\n", argv[4]);
          return -1;
        }
      }

      if (time_ms <= 0 || interval_ms <= 0) {
        LibXR::STDIO::Printf(
            "Error: time_ms and interval_ms must be > 0.\r\n");
        return -1;
      }

      int elapsed = 0;
      while (elapsed < time_ms) {
        print_once(view);
        LibXR::Thread::Sleep(interval_ms);
        elapsed += interval_ms;
      }
      return 0;
    }

    if (strcmp(argv[1], "once") == 0) {
      if (argc > 3) {
        LibXR::STDIO::Printf("Error: Too many arguments for once.\r\n");
        return -1;
      }
      DebugView view = DebugView::FULL;
      if (argc == 3 && !parse_view(argv[2], &view)) {
        LibXR::STDIO::Printf("Error: Unknown view '%s'.\r\n", argv[2]);
        return -1;
      }
      print_once(view);
      return 0;
    }

    DebugView direct_view = DebugView::FULL;
    if (argc == 2 && parse_view(argv[1], &direct_view)) {
      print_once(direct_view);
      return 0;
    }

    LibXR::STDIO::Printf("Error: Unknown command '%s'.\r\n", argv[1]);
    return -1;
  }

 private:
  /**
   * @brief RamFS静态命令回调包装
   * @param self InfantryLauncher对象指针
   * @param argc 命令参数个数
   * @param argv 命令参数数组
   * @return int 命令执行结果
   * @details 将静态回调转发到对象成员函数 DebugCommand。
   */
  static int CommandFunc(InfantryLauncher *self, int argc, char **argv) {
    return self->DebugCommand(argc, argv);
  }

 private:
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

    launcher_state_ = launcher_cmd_.isfire ? LauncherState::NORMAL
                                           : LauncherState::STOP;
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
        trig_mode_ = TRIGMODE::RELAX;
        press_continue_ = false;
        break;

      case LauncherState::STOP:
        trig_mode_ = TRIGMODE::SAFE;
        press_continue_ = false;
        break;

      case LauncherState::NORMAL:
        if (!last_fire_notify_) {
          fire_press_time_ = now;
          press_continue_ = false;
          trig_mode_ = TRIGMODE::SINGLE;
        } else {
          if (!press_continue_ &&
              (now - fire_press_time_).ToSecondf() >
                  launcher::param::LONG_PRESS_THRESHOLD_SEC) {
            press_continue_ = true;
          }
          trig_mode_ = press_continue_ ? TRIGMODE::CONTINUE : TRIGMODE::SINGLE;
        }
        break;

      case LauncherState::JAMMED:
        trig_mode_ = TRIGMODE::JAM;
        break;
    }
  }

  /**
   * @brief 更新拨盘目标角度
   * @param now 当前时间戳
   * @details 根据 TRIGMODE 生成目标角度；卡弹模式下周期切换正反向退弹角度。
   */
  void UpdateTriggerSetpoint(LibXR::MillisecondTimestamp now) {
    switch (trig_mode_) {
      case TRIGMODE::RELAX:
      case TRIGMODE::SAFE:
        target_trig_angle_ = trig_angle_;
        shoot_active_ = false;
        break;

      case TRIGMODE::SINGLE:
        if (last_trig_mode_ == TRIGMODE::SAFE ||
            last_trig_mode_ == TRIGMODE::RELAX ||
            last_trig_mode_ == TRIGMODE::JAM) {
          target_trig_angle_ = trig_angle_ + launcher::param::TRIG_STEP;
          shoot_active_ = true;
        }
        break;

      case TRIGMODE::CONTINUE: {
        float trig_freq = std::max(trig_freq_, 1e-3f);
        float interval_s = 1.0f / trig_freq;
        float since_last = (now - last_trig_time_).ToSecondf();
        if (since_last >= interval_s) {
          target_trig_angle_ = trig_angle_ + launcher::param::TRIG_STEP;
          last_trig_time_ = now;
          shoot_active_ = true;
        }
      } break;

      case TRIGMODE::JAM: {
        jam_keep_time_s_ = (now - last_jam_time_).ToSecondf();
        if (jam_keep_time_s_ >= launcher::param::JAM_TOGGLE_INTERVAL_SEC) {
          if (last_trig_mode_ != TRIGMODE::JAM) {
            is_reverse_ = true;
          }
          target_trig_angle_ =
              trig_angle_ +
              (is_reverse_ ? -2.0f * launcher::param::TRIG_STEP
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
      number_ += 1.0f;
      return;
    }

    if ((now - fire_press_time_).ToSecondf() > 0.2f) {
      shoot_active_ = false;
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

    heat_limit_.current_heat += heat_limit_.single_heat * heat_limit_.launched_num;
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

    if (residuary_heat <= heat_limit_.single_heat * heat_limit_.heat_threshold) {
      float safe_freq = referee_data_.heat_cooling / heat_limit_.single_heat;
      float ratio =
          residuary_heat / (heat_limit_.single_heat * heat_limit_.heat_threshold);
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
   * @brief 失控处理
   * @details 复位状态机与PID并关闭输出，确保发射机构进入安全状态。
   */
  void LostCtrl() {
    mutex_.Lock();
    launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
    launcher_state_ = LauncherState::RELAX;
    trig_mode_ = TRIGMODE::RELAX;

    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();

    target_trig_angle_ = trig_angle_;
    shoot_active_ = false;
    press_continue_ = false;
    launcher_cmd_.isfire = false;

    motor_trig_->Disable();
    motor_fric_0_->Relax();
    motor_fric_1_->Relax();
    mutex_.Unlock();
  }

  /**
   * @brief 拨盘控制解算
   * @param out_trig 拨盘控制输出
   * @param target_trig_angle 拨盘目标角度
   * @param dt 控制周期
   * @details 角度环生成参考速度，速度环生成最终控制输出，并进行速度限幅。
   */
  void TrigControl(float &out_trig, float target_trig_angle, float dt) {
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
  void FricControl(float &out_fric_0, float &out_fric_1, float target_rpm,
                   float dt) {
    out_fric_0 = pid_fric_0_.Calculate(target_rpm, param_fric_0_.velocity, dt);
    out_fric_1 = pid_fric_1_.Calculate(target_rpm, param_fric_1_.velocity, dt);

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_SAFE) {
      pid_fric_0_.SetOutLimit(1.5f);
      pid_fric_1_.SetOutLimit(1.5f);
    }
  }

  LauncherEvent launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
  LauncherState launcher_state_ = LauncherState::RELAX;
  TRIGMODE trig_mode_ = TRIGMODE::RELAX;
  TRIGMODE last_trig_mode_ = TRIGMODE::RELAX;

  RefereeData referee_data_{.heat_limit = 0.0f, .heat_cooling = 0.0f};
  HeatLimit heat_limit_{
      .single_heat = 0.0f,
      .launched_num = 0.0f,
      .current_heat = 0.0f,
      .heat_threshold = 0.0f,
      .allow_fire = true,
  };

  private:
  RMMotor *motor_fric_0_;
  RMMotor *motor_fric_1_;
  RMMotor *motor_trig_;

  Motor::Feedback param_fric_0_{};
  Motor::Feedback param_fric_1_{};
  Motor::Feedback param_trig_{};

  LibXR::PID<float> pid_trig_angle_;
  LibXR::PID<float> pid_trig_sp_;
  LibXR::PID<float> pid_fric_0_;
  LibXR::PID<float> pid_fric_1_;

  LauncherParam param_;
  CMD::LauncherCMD launcher_cmd_{};

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

  LibXR::Thread thread_;
  LibXR::Mutex mutex_;
  CMD *cmd_;

  LibXR::Topic shoot_waiting_ = LibXR::Topic::CreateTopic<float>("shoot_dt");
  LibXR::Topic shoot_number_ = LibXR::Topic::CreateTopic<float>("shoot_number");
  LibXR::Topic shoot_freq_ = LibXR::Topic::CreateTopic<float>("trig_freq");

  LibXR::RamFS::File cmd_file_;
};

#ifdef DEBUG
#define INFANTRY_LAUNCHER_DEBUG_IMPL
#include "InfantryLauncherDebug.inl"
#undef INFANTRY_LAUNCHER_DEBUG_IMPL
#endif
