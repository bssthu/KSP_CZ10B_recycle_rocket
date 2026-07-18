# 长征十号乙海上网系回收：KSP 复现实验

本项目在 KSP 1.12.5 中复现“二级继续入轨、一级返回海上平台、移动钢索四点捕获并弹性吸能”的完整自动任务。kOS 负责一级飞控；二级 kOS/插件逻辑负责入轨；`CZ10BNetRecovery.dll` 补足原版 KSP 缺少的移动柔索、四钩捕获和弹簧阻尼承载。

## 当前结论

当前代码 **不合格，处于修复前状态**。已知硬失败为：

- 一级最终落入海中，而不是由钢索持续承载；
- 10 km 以下出现开关式 PWM；
- 喷管轴与地面相对速度方向夹角超过 30°；
- 旧验收器没有把一级 `Splashed`、50 km以下全部动力物理帧的30°限制和60 s捕获保持完整纳入总判定。

旧文档和旧日志中的 `SEA_MISSION_TEST_PASS` 是历史旧判据结果，不代表满足当前任务。唯一强制口径是[强制约束与验收基线](docs/mandatory-mission-constraints.md)。任何一项失败、未测或只做了离线验证，完整任务都不能宣称通过。

## 已确认的目标方案

- 火箭位于发射台中心；回收船先部署，稳定后等待10 s；一级从0%油门点火，5 s线性升至100%，TT18-A只在实测TWR严格大于1.05后释放。
- 重力转向约500 m开始；35 km时目标俯仰为0°～15°。一级以不超过20%推进剂余量分离，分离前实际卸载一级推力。
- 二级在分离后1 s内点火，推力设定严格大于10%，首次连续燃烧严格超过10 s，首次关机时距一级严格超过10 km，最终进入95～110 km远地点、90～105 km近地点轨道。
- 一级稳定滑行越过远地点；下降到50 km后进入表面逆行姿态；下降穿越40 km时执行一次连续入口减速，水平地速降至不超过1000 m/s后立即关机。
- 入口减速与主制动之间以无动力下降为主，只允许2～3个预先声明的检查点按偏差触发一次连续修正。
- 主制动按连续75%最大推力规划，轨迹修正可连续提高到100%；不允许开关式PWM。
- 2 km门槛为下降速度150～200 m/s、水平地速不超过5 m/s、挂钩—网口水平误差不超过10 m，并始终满足逐物理帧30.0°喷管约束。
- 一级从框上方直接进入，钢索连续随动并按高度线性收口；捕获后约50°弹性下挠。一级任何零件不得触海或进入`Splashed`，四点捕获必须连续稳定60 s。

Kerbin重力、大气、DragCube计算和后续轨迹预测采用的混合模型见[Kerbin重力、大气与气动混合模型](docs/ksp-physics-and-hybrid-model.md)。

## 文档入口

- [强制约束与验收基线](docs/mandatory-mission-constraints.md)
- [Kerbin重力、大气与气动混合模型](docs/ksp-physics-and-hybrid-model.md)
- [事件、部件与Mod调研](docs/research-2026-07-10.md)
- [完整自动飞行控制方案](docs/automatic-flight-control-scheme.md)
- [控制算法与日志迭代](docs/control-and-ai-loop.md)
- [游戏内拼装与分阶试飞](docs/build-and-test.md)
- [全自动实机观看与重播](docs/watch-full-mission.md)
- [真实海上任务历史记录](docs/sea-mission-acceptance.md)
- [KSC同场隔离验收历史](docs/full-mission-acceptance.md)
- [kOS低空闭环专项验收](docs/kos-hover-acceptance.md)
- [kOS主程序](Ships/Script/cz10b/main.ks)
- [控制参数](Ships/Script/cz10b/config.ks)
- [捕获物理插件](src/NetRecovery/ModuleCatchNet.cs)

## 验证层级

1. 构建、kOS语法检查和离线回归只用于快速排除错误。
2. 穿网、低空、同场两级任务是专项诊断，不能代替真实海上任务。
3. 最终结论必须来自一次不中断的完整KSP实机任务，并逐项给出强制基线证据。
4. 单独的总PASS字符串、旧日志、离线250/250或“画面看起来成功”都不是充分条件。

## 本地开发检查

```powershell
dotnet build .\src\NetRecovery\NetRecovery.csproj -c Release
python .\tools\simulate_controller.py
python .\tools\analyze_telemetry.py
```

这些命令不会给出完整任务合格结论。实机启动、观察和日志位置见[全自动实机观看与重播](docs/watch-full-mission.md)。
