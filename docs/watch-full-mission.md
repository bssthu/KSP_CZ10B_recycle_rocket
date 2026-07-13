# 全自动实机观看与重播

当前版本没有额外的游戏内启动面板。完整海上演示由 `CZ10BNetRecovery` 的一次性启动标记触发：KSP 启动到主菜单后会自动创建沙盒、进入航天中心、把时间推进到下一次 KSC 当地 06:00、装载验收飞船、把回收平台部署到海面，并从一级点火开始运行完整任务。正常观看不需要 VesselMover，也不需要在 VAB 手工拼装。

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
3. 插件把时间推进到下一次 KSC 当地 06:00，再自动装载 `CZ10B Full Mission Recovery Test`。
4. 初始火箭位于发射台中心，回收船在其侧面；组合载具先等待 8 秒完成物理解包，再分离回收船，将其部署到纬度约 −0.0972°、经度约 −22.281° 的实测自然落区（距一级约 529 km）。部署时同时固定经纬度、当地旋转速度和甲板朝天姿态。
5. 回收船完成水面物理解包后，在距火箭超过 120 km 时转为打包待机；一级随后点火，两个 TT18-A 在实测推重比超过 1.02 后释放起飞。一级接近回收船 120 km 时，平台自动恢复真实水面物理。
6. 一级在 500 m 开始转弯，到 25 km 时达到 15° 仰角；稠密大气段速度上限为 720 m/s。实测约在 40.2 km、一级余量 19.97% 时分离。二级先把远地点抬到约 100 km，再在远地点附近圆化；最终轨道约为 111.6 × 90.0 km。
7. 分离前一级发动机推力限制先置零，分离完成后才恢复；一级零油门上升滑行采用纯角速度阻尼，本轮最大角速度 0.60°/s。下降到 50 km 后喷管平滑转向速度方向，40 km 全推力制动到水平速度不超过 1000 m/s。
8. 30 km 是弹道规划交接点；一级沿实测大气阻力预测落区滑行，到自杀点火门限才建立一次固定、高度索引的轨迹，不做连续移动瞄准点的小修正。1 km/700 m 内切入停车速度走廊，300 m 内切入慢速阻尼；挂点距网面 45 m 内进行最多 12 秒最终对准，之后以约 1.5 m/s 单调下穿并只阻尼残余横移。
9. 挂点进入网面上方 100 m 且位于 24 m × 20 m 框内后，四条索立即闭合并持续跟随挂点中心。本轮以垂直 −1.46 m/s、横向 0.40 m/s、倾角 1.2° 四点捕获；捕获后四个绞盘锚点按每个物理步连续放绳到 11.92 m，形成约 50° 索角，且关节反力不会使回收船侧翻或把一级甩出。
10. 捕获后场景保持运行，塔架上的静态收拢抱夹可作为转运固定机构的外观参考，方便环绕观察。

当前流程从进入 Flight 到稳定通过约 620 秒任务时间；连同游戏加载所需的墙钟时间取决于磁盘和近场物理帧率，性能较低时会明显更久。应以屏幕流程或 `SEA_MISSION_TEST_PASS` 为准，不要因超过固定分钟数而中断。

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

完整成功标记为下列一行；当前判据还要求放绳完成后连续 8 秒满足一级垂直速度不超过 4 m/s、水平速度不超过 3 m/s、角速度不超过 12°/s，并且平台倾角不超过 3°：

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
