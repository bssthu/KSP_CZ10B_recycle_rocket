# 完整两级返航验收

当前 `CZ10B Full Mission Recovery Test` 是八零件自动验收器：回收平台、任务发射架、独立 stock Mainsail、一级油箱/控制段、TD-25、集成二级和两个 stock TT18-A。C# 观察器负责当地 06:00 场景、海上平台部署/远距加载、测试发射架释放、活动载具切换、二级入轨保护和物理门槛记录；所有一级点火、姿态、油门、级间分离与返航指令仍来自 `cz10b/main.ks`。完整的当前控制链见[完整自动飞行控制方案](automatic-flight-control-scheme.md)。

0.3.1 在 KSP 1.12.5 KSC 同场 Flight 场景的隔离验收结果：

- 一级发动机产生实际推力并越过 2 km 验收高度。
- 载具升至 18 km 后显式点燃二级、触发 TD-25，二级成为独立载具。
- 一级在平台超出物理加载范围时使用载具级位置/速度，完成高空 boost-back；平台重新载入后切换为网零件精确位置。
- 末段在横误差大于 5 m 时悬停归中；物理层拒绝部分挂钩，只有全部四个挂钩位于网窗才建立柔性关节。
- 捕获瞬间日志：4 hooks，垂直相对速度 −0.64 m/s，横向相对速度 1.27 m/s，倾角 5.3°。
- 捕获后稳定 8 秒，最终日志：`MISSION_TEST_PASS powered=True high=True separated=True hook=Captured`。

最终任务遥测为 `Ships/Script/cz10b/telemetry.csv`。当前 Python 3.10 与 3.13 的 250 组扰动回归均为 250/250 捕获。

这个验收发生在 KSC 同场平台，目的是隔离并证明两级与返航闭环。0.4.0 已进一步通过自动真实海面全任务；详见 [真实海上全任务验收](sea-mission-acceptance.md)。VesselMover 仍可用于手工布置和外观调试，但不再是自动验收的必要条件。
