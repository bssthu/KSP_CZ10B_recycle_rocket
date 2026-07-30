# 长征十号乙海上网系回收：KSP 复现实验

本项目在 KSP 1.12.5 中复现“二级继续入轨、一级返回海上平台、移动钢索四点捕获并弹性吸能”的完整自动任务。kOS 负责一级飞控；二级 kOS/插件逻辑负责入轨；`CZ10BNetRecovery.dll` 补足原版 KSP 缺少的移动柔索、四钩捕获和弹簧阻尼承载。

## 当前结论

Run333 已在 KSP 1.12.5 中完成当前基线下第一轮不中断的恢复级完整
任务，自动逐项矩阵结论为 `tier=RECOVERED`。2 km 插值状态为
`191.45 m/s / 4.47 m/s / 29.51 m`，因此 G-05N 名义位置指标未通过，
但 G-05R 通过；随后在挂钩高度约 292 m 实际进入
`6.79 m / 3.45 m/s / 0.79°` 的 G-05C 严格提交集。最终以四钩、
`vertical=-7.04 m/s / lateral=1.75 m/s / tilt=0.9°` 捕获，连续稳定
60.0 s，无触海、无捕获完整性异常，G-04 全动力段峰值为 27.442°。

该结果证明恢复准入状态可以由现有有界控制修回，但不能回填为 G-05N
名义通过。唯一强制口径仍是[强制约束与验收基线](docs/mandatory-mission-constraints.md)；
后续代码、构型或外观变化都必须重新执行完整任务，视觉结论仍需按观看
清单人工复核，不能只引用本轮 PASS 字符串。

## 已确认的目标方案

- 火箭位于发射台中心；回收船先部署，稳定后等待10 s；一级从0%油门点火，5 s线性升至100%，TT18-A只在实测TWR严格大于1.05后释放。
- 重力转向约500 m开始；35 km时目标俯仰为0°～15°。一级以不超过20%推进剂余量分离，分离前实际卸载一级推力。
- 二级在分离后1 s内点火，推力设定严格大于10%，首次连续燃烧严格超过10 s，首次关机时距一级严格超过10 km，最终进入95～110 km远地点、90～105 km近地点轨道。
- 一级稳定滑行越过远地点；下降到50 km后进入表面逆行姿态；下降穿越40 km时执行一次连续入口减速，水平地速降至不超过1000 m/s后立即关机。
- 入口减速与主制动之间以无动力下降为主，只允许2～3个预先声明的检查点按偏差触发一次连续修正。
- 主制动按连续75%最大推力规划，轨迹修正可连续提高到100%；不允许开关式PWM。
- 2 km采用分层验收：`5 m/s / 10 m`保留为名义质量目标；未达到名义目标时，只有下降速度仍为150～200 m/s、水平地速不超过10 m/s、误差不超过30 m且状态可恢复，才允许继续作为捕获候选。取消位置追踪仍必须等到实际达到`3.9 m/s / 8 m / 14.7°`严格提交集合。
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
