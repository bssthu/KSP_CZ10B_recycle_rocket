# 长征十号乙海上网系回收：KSP 复现实验

本项目把 2026-07-10 长征十号乙的动作拆成两层：kOS 负责一级火箭的发射、分离、返航与相对回收船制导；`CZ10BNetRecovery.dll` 负责 KSP 原生物理缺少的“挂钩接触柔索、弹簧阻尼吸能”环节。

## 当前状态

- 已确认本机游戏为 KSP 1.12.5，且装有 Breaking Ground / Making History。
- 已安装官方 kOS 1.6.0.1 到游戏目录。
- 已编译、安装十个功能/验收零件；除独立捕获网、挂钩、单件回收平台、单件一级和单件二级外，还包含低空、穿网与完整任务所需的测试夹具。
- 已安装 kOS 发射/回收脚本；官方解析器对全部 `.ks` 文件检查通过。
- 0.6.0 在本机无测试标记启动到主菜单的生产冒烟测试通过：十个零件均由 PartLoader 编译，全局 `ERR=0`、`EXC=0`，且不会自动进入任务。
- 当前开发版把下降中段改为每 0.5 秒按实测位置、速度和燃料重算的滚动轨迹；只有高度低于 1 km 且距船小于 700 m 才进入横向制动走廊，距船 300 m 后永久锁入线性速度场。锁内速度环增益按实测 1.5 秒姿态滞后降到 0.35，避免穿越船心后反向高速追逐；垂直近场 PI 仍到 18 m 才接管。
- 2026-07-13 的最终完整海上实机已在静音、KSC 当地中午条件下通过 `SEA_MISSION_TEST_PASS`：一级在 34.4 km、1921.3 m/s、余量 19.81% 时分离，二级进入约 120 × 90 km 轨道；一级末段距离单调收敛，无中心反复穿越，以垂直 −1.48 m/s、横向 0.34 m/s、倾角 0° 四点捕获。四个独立绞盘锚点连续放缆到 17.32 m，随后一级速度归零、平台倾斜 0.03°，严格稳定窗口连续超过 8 秒，落地余量 0.43%。
- 已生成并安装五个模板；`CZ10B kOS Hover Recovery Test` 与 `CZ10B Full Mission Recovery Test` 均已由 KSP 1.12.5 原生加载。
- 0.6.0 已在真实海面 Flight 场景完成“火箭居中、船先部署、延时点火、真实夹具释放、45° Kerbin 尺度程序转弯、稳定越过远地点、单向返航、固定姿态再入、固定轨迹下降、连续随动闭索和柔性捕获”的完整链路。最终以垂直 −0.63 m/s、横向 0.09 m/s、倾角 0° 四点捕获，柔索受载下挠 0.62–0.63 m，并稳定到 `SEA_MISSION_TEST_PASS`。
- 已在独立 `CZ10BRecoveryTest` 沙盒完成真实 Flight 穿网测试：TD-25 分离后形成两个载具，四个柔性关节在法向速度 -1.18 m/s、横速 0、倾角 0° 时捕获，并稳定保持到 `DROP_TEST_PASS`。

游戏已按用户要求设置 `MASTER_VOLUME = 0`。
自动演示会在进入航天中心后把游戏时间推进到下一次 KSC 当地中午，便于全程观察。

## 项目入口

- [事件与 Mod 调研](docs/research-2026-07-10.md)
- [游戏内拼装与试飞](docs/build-and-test.md)
- [控制算法与日志迭代](docs/control-and-ai-loop.md)
- [真实海上全任务验收](docs/sea-mission-acceptance.md)
- [全自动实机观看与重播](docs/watch-full-mission.md)
- [kOS 主程序](Ships/Script/cz10b/main.ks)
- [控制参数](Ships/Script/cz10b/config.ks)
- [捕获物理插件](src/NetRecovery/ModuleCatchNet.cs)

## 本地验证命令

```powershell
dotnet build .\src\NetRecovery\NetRecovery.csproj -c Release
python .\tools\simulate_controller.py
python .\tools\analyze_telemetry.py
```

五个模板可由 `tools/install.ps1` 复制到指定存档。按“地面穿网测试 → kOS 低空闭环 → KSC 完整任务 → 自动海上完整任务”的顺序复现，可以把零件方向、控制轴、TWR、远距离加载和水面物理五类问题分开；VesselMover 保留为手工布置与外观调试工具。
