# Launcher

## 1. 模块作用
发射机构总控模块。通过模板封装 HeroLauncher 或 InfantryLauncher 实现。

## 2. 主要函数说明
1. Launcher<>::GetEvent / EventHandler: 对外事件接口。
2. HeroLauncher::ThreadFunction / Update / Control / HeatLimit / SetMode。
3. InfantryLauncher::RunStateMachine / TrigControl / UpdateHeatControl / SetMode。
4. LostCtrl: 丢控保护处理。

## 3. 接入步骤
1. 添加模块并在 template_args 选择 LauncherType。
2. 绑定摩擦轮与拨弹电机，配置 PID 与机构参数。
3. 先验证模式切换，再验证单发与连发流程。

标准命令流程：
    xrobot_add_mod Launcher --instance-id launcher
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 4. 配置示例（YAML）
module: Launcher
entry_header: Modules/Launcher/Launcher.hpp
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
      fric_setpoint_speed: [4950.0, 3820.0]
      fric_num: 2
      trig_gear_ratio: 19.2032
      num_trig_tooth: 6
      trig_freq_: 0.0
  - cmd: '@&cmd'
template_args:
  - LauncherType: HeroLauncher

InfantryLauncher 可通过 `launcher_param.fric_num` 指定使用的摩擦轮数量，`fric_setpoint_speed` 取前 N 项，未使用的摩擦轮 PID 将被跳过。

## 5. 依赖与硬件
Required Hardware:
  - dr16
  - can

Depends:
  - qdu-future/CMD
  - qdu-future/RMMotor

## 6. 代码入口
Modules/Launcher/Launcher.hpp
