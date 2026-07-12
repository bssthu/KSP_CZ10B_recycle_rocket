# 长征十号乙海上网系回收：KSP 复现实验

本项目把 2026-07-10 长征十号乙的动作拆成两层：kOS 负责一级火箭的发射、分离、返航与相对回收船制导；`CZ10BNetRecovery.dll` 负责 KSP 原生物理缺少的“挂钩接触柔索、弹簧阻尼吸能”环节。

## 当前状态

- 已确认本机游戏为 KSP 1.12.5，且装有 Breaking Ground / Making History。
- 已安装官方 kOS 1.6.0.1 到游戏目录。
- 已编译、安装十个功能/验收零件；除独立捕获网、挂钩、单件回收平台、单件一级和单件二级外，还包含低空、穿网与完整任务所需的测试夹具。
- 已安装 kOS 发射/回收脚本；官方解析器对全部 `.ks` 文件检查通过。
- 0.5.0 在本机无测试标记启动到主菜单的生产冒烟测试通过：十个零件均由 PartLoader 编译，全局 `ERR=0`、`EXC=0`，且不会自动进入任务。
- 终端控制器在 Python 3.10 与 3.13 下分别完成 250 组确定性扰动测试，两次均达到 250/250 进入捕获包线。
- 已生成并安装五个模板；`CZ10B kOS Hover Recovery Test` 与 `CZ10B Full Mission Recovery Test` 均已由 KSP 1.12.5 原生加载。
- 0.5.0 已在真实海面 Flight 场景完成“船先部署、延时点火、夹具释放、常规重力转弯、18 km 两级分离、越过远地点、返航/再入制动、闭索和四挂钩捕获”的完整链路；捕获瞬间为垂直 −0.62 m/s、横向 0.02 m/s、倾角 0°，随后稳定到 `SEA_MISSION_TEST_PASS`。
- 已在独立 `CZ10BRecoveryTest` 沙盒完成真实 Flight 穿网测试：TD-25 分离后形成两个载具，四个柔性关节在法向速度 -1.18 m/s、横速 0、倾角 0° 时捕获，并稳定保持到 `DROP_TEST_PASS`。

游戏已按用户要求设置 `MASTER_VOLUME = 0`。

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

五个模板已复制到 `C:\Projects\Kerbal Space Program\saves\abc\Ships\VAB`。按“地面穿网测试 → kOS 低空闭环 → KSC 完整任务 → 自动海上完整任务”的顺序复现，可以把零件方向、控制轴、TWR、远距离加载和水面物理五类问题分开；VesselMover 保留为手工布置与外观调试工具。
