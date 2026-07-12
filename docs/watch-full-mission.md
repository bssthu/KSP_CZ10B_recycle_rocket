# 全自动实机观看与重播

当前版本没有额外的游戏内启动面板。完整海上演示由 `CZ10BNetRecovery` 的一次性启动标记触发：KSP 启动到主菜单后会自动创建沙盒、进入航天中心、装载验收飞船、把回收平台部署到海面，并从一级点火开始运行完整任务。正常观看不需要 VesselMover，也不需要在 VAB 手工拼装。

## 一键启动完整海上演示

先关闭所有 KSP 实例，再在 PowerShell 中执行：

```powershell
$ksp = 'C:\Projects\Kerbal Space Program'
$pluginData = Join-Path $ksp 'GameData\CZ10BRecovery\PluginData'
$volume = (Select-String `
  -LiteralPath (Join-Path $ksp 'settings.cfg') `
  -Pattern '^\s*MASTER_VOLUME\s*=' | Select-Object -First 1).Line.Trim()

if ($volume -ne 'MASTER_VOLUME = 0') {
  throw "为避免启动声音，已中止：$volume"
}

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
4. 初始火箭位于发射台中心，回收船在其西侧；组合载具先等待 8 秒完成物理解包，再分离回收船，将其部署到 KSC 以东约 36.6 km 的深水海面。
5. 回收船移动完成 10 秒后一级点火；两个 TT18-A 继续抱持，实测推重比超过 1.02 后才释放起飞。
6. 火箭按 Kerbin 较小半径和 70 km 大气层修正为 45° 常规程序转弯，在约 18 km 完成两级分离；上面级沿分离前方向继续飞行。
7. 一级保持分离姿态稳定滑行，下降速度达到 −25 m/s 后才执行一次平滑翻转；单向返航点火约 45 秒，随后用 6 秒无动力转到固定竖直姿态并执行一次再入制动，不再追逐快速变化的速度方向。
8. 10 km 以下先飞固定到达时间的受约束三次轨迹，500 m 以下才切换近场 PID；正常任务保持连续下降，不以悬停平移代替轨迹制导。
9. 一级进入塔架后四条索一边闭合一边持续跟随挂点中心；索网闭合且跟踪稳定后一级以约 0.63 m/s 穿网，捕获索随后受载向下挠曲约 0.6 m。
10. 捕获后场景保持运行，塔架上的静态收拢抱夹可作为转运固定机构的外观参考，方便环绕观察。

0.6.0 实测从进入 Flight 到稳定通过约 7 分钟（任务 UT 约 415 秒）。连同游戏加载所需的墙钟时间取决于磁盘和近场物理帧率，性能较低时会明显更久；应以屏幕流程或 `SEA_MISSION_TEST_PASS` 为准，不要因超过固定分钟数而中断。

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
