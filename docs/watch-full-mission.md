# 全自动实机观看与重播

## 1. 当前警告

当前控制代码尚未通过[强制约束与验收基线](mandatory-mission-constraints.md)。最近观察到一级落海、10 km以下PWM和喷管夹角超过30°。下面的启动方式当前只能用于开发观察和复现失败，不能据此宣称任务通过。

旧版`SEA_MISSION_TEST_PASS`字符串不是当前成功标志。最终合格必须由逐项验收矩阵和人工视觉检查共同确认，并完成捕获后60 s保持。

## 2. 一键启动海上任务

先关闭所有KSP实例，再在PowerShell执行：

```powershell
$ksp = 'C:\Projects\Kerbal Space Program'
$pluginData = Join-Path $ksp 'GameData\CZ10BRecovery\PluginData'
$volume = (Select-String `
  -LiteralPath (Join-Path $ksp 'settings.cfg') `
  -Pattern '^\s*MASTER_VOLUME\s*=' | Select-Object -First 1).Line.Trim()

if ($volume -ne 'MASTER_VOLUME = 0') {
  throw "为避免自动启动声音，已中止：$volume"
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

`CZ10BShowcase`是一次性演示沙盒名，启动器会重建同名沙盒。不要替换为需要保留的正常存档。标记文件读取后会自动删除；重播需要退出KSP并重新创建标记。

## 3. 修复后应看到的目标流程

1. KSP进入独立沙盒并把时间推进到下一次KSC当地06:00。
2. 火箭位于发射台中心，回收船位于侧面并先部署到海上。
3. 船稳定后等待完整10 s；一级以0%油门点火，5 s线性升到100%。
4. 两个TT18-A与一级和地面保持连接，实测TWR严格大于1.05后正常释放。
5. 一级约500 m开始转弯，35 km时俯仰位于0°～15°；分离余量不超过20%。
6. 分离前一级实际推力卸载；两级无回撞。二级1 s内点火，推力设定>10%，首次燃烧>10 s，首次关机时距一级>10 km，最终进入95～110 × 90～105 km轨道。
7. 一级稳定滑行越过远地点；下降到50 km后转为表面逆行。
8. 下降穿越40 km时执行一次连续入口燃烧，水平地速≤1000 m/s后立即关机。
9. 中间主要无动力下降，只在2～3个预定检查点按偏差执行一次连续修正。
10. 计算得到的点火位置开始连续75%标称主制动；没有0/75%开关式PWM，所有动力帧喷管夹角≤30.0°。
11. 海拔2 km时下降速度150～200 m/s、水平≤5 m/s、误差≤10 m；之后连续、近似竖直下降且不反弹。
12. 四索连续跟踪并线性收口，一级从框上方四点捕获，钢索连续下挠到约50°。
13. 一级全程不触海、不进入`Splashed`；捕获后持续稳定至少60 s。

当前代码若没有呈现上述流程，应记录失败阶段并停止“成功”结论，而不是用旧屏幕提示覆盖观察结果。

## 4. 观看时不要干预

- 保持物理时间倍率`1×`，不要时间加速。
- 不要按空格或手动修改油门、SAS、RCS、姿态和分级。
- 不要用`[`、`]`切换载具；摄像机尽量跟随一级。
- 可以旋转/缩放摄像机、查看地图或隐藏界面，但不能改变载具状态。
- `AG10`只用于确认失败后的安全中止。

任何人工控制都会令该次运行失去完整自动任务验收资格。

## 5. 必须观察的视觉项目

- 发射前：火箭居中、回收船先移走、TT18-A上下连接、一级发动机无断口/罩壳、二级顶部整流外形连续。
- 点火：0%起始和5 s爬升、正常喷焰/烟迹、导流槽烟尘、TT18-A释放动画。
- 上升与分离：无额外大机动、无增长振荡、两级无碰撞、二级立即持续点火。
- 再入：50 km后平滑逆行、40 km一次连续减速、无喷管朝天或大幅反向摆动。
- 10 km以下：无可见脉冲点火、无向上反弹、无多次PID摆动，轨迹在2 km后基本竖直。
- 捕获：火箭先进入框内，钢索逐帧随动和线性收口，无延迟突变/穿模，捕获后约50°弹性下挠。
- 60 s观察：一级不横移出框、不脱索、不下坠、不触海；回收船不侧翻。

## 6. 日志和逐项判定

运行日志：

```text
C:\Projects\Kerbal Space Program\KSP.log
```

kOS遥测：

```text
C:\Projects\Kerbal Space Program\Ships\Script\cz10b\telemetry.csv
```

验收必须逐项输出至少：

- 发射延时、油门爬升、TT18-A释放TWR；
- 重力转向终点、分离余量和分离实际推力；
- 二级点火延迟、推力设定、燃烧时长、首次关机距离和轨道；
- 40 km入口点火/关机、检查点和主制动区间；
- 10 km以下油门转换审计；
- 50 km以下所有动力帧最大喷管夹角；
- 2 km插值状态；
- 垂直速度符号、索网几何、四钩状态、一级最低零件海拔和`Splashed`；
- 捕获后连续60 s状态。

单独出现以下字符串不再足够：

```text
SEA_MISSION_TEST_PASS powered=True high=True separated=True hook=Captured
```

只有全部自动条目为PASS、没有UNVERIFIED，并且视觉清单复核完成，才能按当前基线判定成功。

## 7. 专项演示

以下标记仍可用于隔离调试，但只能报告专项结果：

| 标记文件 | 用途 |
|---|---|
| `launch-drop-test.once` | 四挂点低速穿网与弹性关节 |
| `launch-hover-test.once` | kOS低空推力/位置闭环 |
| `launch-mission-test.once` | KSC同场分级与返航链路 |
| `launch-sea-mission-test.once` | 真实海面完整任务 |

VesselMover可以用于手工外观和专项布置，不得用于在最终完整任务中瞬移一级或拼接成功结果。
