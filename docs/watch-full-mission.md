# 全自动实机观看与重播

当前版本没有额外的游戏内启动面板。完整海上演示由 `CZ10BNetRecovery` 的一次性启动标记触发：KSP 启动到主菜单后会自动创建沙盒、进入航天中心、把时间推进到下一次 KSC 当地中午、装载验收飞船、把回收平台部署到海面，并从一级点火开始运行完整任务。正常观看不需要 VesselMover，也不需要在 VAB 手工拼装。

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
3. 插件把时间推进到下一次 KSC 当地中午，再自动装载 `CZ10B Full Mission Recovery Test`。
4. 初始火箭位于发射台中心，回收船在其侧面；组合载具先等待 8 秒完成物理解包，再分离回收船，将其部署到 KSC 以东约 72.4°（约 759 km）的深水海面。部署时同时固定经纬度、当地旋转速度和甲板朝天姿态。
5. 回收船完成水面物理解包后，在距火箭超过 120 km 时转为打包待机；一级随后点火，两个 TT18-A 在实测推重比超过 1.02 后释放起飞。一级接近回收船 120 km 时，平台自动恢复真实水面物理。
6. 一级在 500 m 开始转弯，到 20 km 时达到 15° 仰角并主要作水平加速；实测约在 34.4 km、总速 1921 m/s、一级余量 19.8% 时分离。二级先把远地点抬到约 100 km，再在远地点附近圆化，并由保护逻辑在近地点 90 km 时关机；典型最终轨道约为 120 × 90 km。
7. 一级保留分离时的水平速度滑行，确认下降速度达到 −25 m/s 后才执行一次平滑发动机朝下翻转，随后保持固定竖直再入姿态；制导使用实测大气阻力修正点火门限，不追逐快速变化的速度方向。
8. 网面上方 15 km 内，每 0.5 秒根据最新高度、垂直/水平速度、相对船位和剩余燃料重算轨迹。垂直通道沿“高空允许高速、接近网面逐步收紧”的下降走廊飞行；横向到高度低于 1 km 且距船小于 700 m 才切换到停车速度走廊，进入 300 m 后永久锁入带姿态滞后阻尼的线性速度场，中心 3 m 内不再追逐位置噪声。垂直近场 PI 仍只在最后 18 m 接管。
9. 挂点进入网面上方 30 m 且位于 24 m × 20 m 框内后，四条索开始闭合并持续跟随挂点中心；进入 20 m 对中范围后最多等待 12 秒，随后以 1.5 m/s 受控穿网，避免因状态字段异常无限悬停。捕获后四个绞盘锚点按物理步连续放绳到约 17.32 m 下挠，形成约 60° 索角，同时不把关节反力反馈成回收船侧翻。
10. 捕获后场景保持运行，塔架上的静态收拢抱夹可作为转运固定机构的外观参考，方便环绕观察。

0.6.0 历史验收从进入 Flight 到稳定通过约 7 分钟。当前较低轨道、远距离弹道和二级入轨流程的任务 UT 接近 700 秒；连同游戏加载所需的墙钟时间取决于磁盘和近场物理帧率，性能较低时会明显更久。应以屏幕流程或 `SEA_MISSION_TEST_PASS` 为准，不要因超过固定分钟数而中断。

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
