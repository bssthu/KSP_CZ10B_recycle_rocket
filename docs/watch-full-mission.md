# 全自动实机观看与重播

当前版本没有额外的游戏内启动面板。完整海上演示由 `CZ10BNetRecovery` 的一次性启动标记触发：KSP 启动到主菜单后会自动创建沙盒、进入航天中心、装载验收飞船、把回收平台部署到海面，并从一级点火开始运行完整任务。正常观看不需要 VesselMover，也不需要在 VAB 手工拼装。

## 一键启动完整海上演示

先关闭所有 KSP 实例，再在 PowerShell 中执行：

```powershell
$ksp = 'C:\Projects\Kerbal Space Program'
$pluginData = Join-Path $ksp 'GameData\CZ10BRecovery\PluginData'

New-Item -ItemType Directory -Force -Path $pluginData | Out-Null

Set-Content `
  -LiteralPath (Join-Path $pluginData 'launch-sea-mission-test.once') `
  -Value 'CZ10BShowcase' `
  -Encoding ASCII `
  -NoNewline

Start-Process `
  -FilePath (Join-Path $ksp 'KSP_x64.exe') `
  -WorkingDirectory $ksp
```

`CZ10BShowcase` 是专用演示存档名。启动器会重新创建同名沙盒，因此不要把这里改成需要保留的正常游戏存档名。标记文件在读取后会自动删除；要重播时，退出 KSP 并重新执行上述命令。

## 自动流程

1. KSP 加载到主菜单。
2. 插件创建 `CZ10BShowcase` 沙盒并进入航天中心。
3. 自动装载 `CZ10B Full Mission Recovery Test`。
4. 回收平台部署到 KSC 以东约 6.3 km 的深水海面并释放水面物理。
5. kOS 点燃一级、执行上升，在约 18 km 完成两级分离。
6. 一级执行 boost-back、再入制动、150 m 以下高度保持与横向对中。
7. 四个挂点低速进入缆网；捕获后场景保持运行，方便环绕观察。

从进入 Flight 到捕获约 8 分钟；连同游戏加载通常需要约 10 分钟。

## 观看时不要干预的项目

- 保持物理时间倍率为 `1×`，不要使用时间加速。
- 不要按空格，不要手动改变油门、SAS、RCS 或姿态。
- 尽量不要用 `[`、`]` 切换载具，保持摄像机跟随一级。
- 可以自由旋转/缩放摄像机，按 `M` 查看地图，按 `F2` 隐藏界面。
- 动作组 `AG10` 是手动中止开关。
- 捕获成功时屏幕会显示 `NET CAPTURE`；看完后按 `Esc` 返回航天中心或退出游戏。

游戏当前按用户要求保持 `MASTER_VOLUME = 0`。如需声音，可在 KSP 设置中自行恢复主音量。

## 日志与成功标记

运行日志：

```text
C:\Projects\Kerbal Space Program\KSP.log
```

kOS 遥测：

```text
C:\Projects\Kerbal Space Program\Ships\Script\cz10b\telemetry.csv
```

完整成功标记为：

```text
SEA_MISSION_TEST_PASS powered=True high=True separated=True hook=Captured
```

## 其他自动演示

将 PowerShell 中的标记文件名替换为以下名称，可以运行分阶段演示；文件内容仍建议使用独立沙盒名：

| 标记文件 | 演示内容 |
|---|---|
| `launch-drop-test.once` | 四挂点低速穿网 |
| `launch-hover-test.once` | kOS 低空悬停、平移与归中 |
| `launch-mission-test.once` | KSC 同场平台上的完整两级任务 |
| `launch-sea-mission-test.once` | 自动部署回收船的完整海上任务 |

VesselMover 工具栏仍可用于手工移动船只或调试自定义飞船，但自动海上演示不依赖它。
