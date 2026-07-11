# kOS 低空闭环验收

`CZ10B kOS Hover Recovery Test` 是完整一级的低空实飞模板。平台上方的 52 m 临时悬架只负责把一级放到初始位置；自动测试器在物理稳定后禁用支架碰撞、直接解耦并把平台设为 KSP 目标。它不写入油门、姿态、发动机或外力。

一级内的 `boot/cz10b-hover-boot.ks` 从 kOS Archive（`0:`）启动 `cz10b/hover_test.ks`。脚本读取目标平台的位置与表面速度，执行水平位置—速度串级、垂直速度 PI、推力矢量与油门闭环，最后由 `ModuleCatchNet` 的同一物理包线完成捕获。

2026-07-12 的真实 KSP 1.12.5 Flight 验收结果：

- 发动机实际推力被观察到，日志峰值约 525 kN；严格验收标志为 `powered=True`。
- 初始下降阶段最大记录速度约 −11.1 m/s，随后连续收敛。
- 捕获瞬间：垂直相对速度 −0.40 m/s，横向相对速度 0.60 m/s，倾角 9.1°。
- 四个虚拟挂钩同时建立柔性关节，捕获后保持稳定 8 秒。
- 最终日志：`HOVER_TEST_PASS ... powered=True`。

遥测写入 `Ships/Script/cz10b/hover-telemetry.csv`，可执行：

```powershell
python .\tools\analyze_telemetry.py 'C:\Projects\Kerbal Space Program\Ships\Script\cz10b\hover-telemetry.csv'
```

验收器只有在“曾产生超过 10 kN 的实际推力、进入捕获状态并稳定 8 秒”三个条件都满足时才会通过，因此纯弹道落入网中不会被误判为 kOS 闭环成功。
