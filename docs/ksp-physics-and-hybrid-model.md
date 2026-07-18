# Kerbin 重力、大气与气动混合模型

## 1. 目的和适用范围

本文记录 KSP 1.12.5 原版 Kerbin 环境的实际计算方式、当前项目所用近似的差距，以及已经确认用于后续轨迹规划的混合模型。它服务于一级从 40 km 入口减速、无动力检查点、75%主制动到 2 km 门槛的预测；不替代完整实机验收。

本机 `GameData` 中没有 FAR、Real Solar System、Kopernicus 或其他替换行星/空气动力学的 Mod，因此以下分析以 KSP 1.12.5 stock `FlightIntegrator`、`CelestialBody`、DragCube 和本机 `Physics.cfg` 为准。若以后安装会修改重力、大气或气动的 Mod，本文参数必须重新核对。

## 2. 核心结论

1. Kerbin 重力可以使用运行时 `Body:MU` 和 `Body:RADIUS` 精确计算，不能在 40 km 轨迹中固定为 `9.81 m/s²`。
2. Kerbin 压力和基础温度由高度曲线定义，不是单一指数大气；实际温度还含太阳位置和纬度修正。
3. 原版阻力保留 `q = ρV²/2` 的形式，但等效 `CdA` 来自每个零件的 DragCube、Mach 曲线、伪雷诺数曲线、迎角和遮挡，不能视为常量。
4. kOS 当前从无动力测速差反推阻力的方向可用于在线校正，但它不能单独提供 40 km 到 2 km 的前视预测。
5. 后续采用混合模型：精确环境公式提供基础预测，KSP 实际零件气动和无动力实测残差持续校正等效阻力。

## 3. Kerbin 重力

### 3.1 惯性系中的公式

KSP stock 在当前天体作用域内使用中心重力。设天体中心到载具的位置为 `r`：

```text
g_vector = -mu * r_vector / |r_vector|^3
g(h)     =  mu / (R + h)^2
```

其中：

- `mu`：当前天体的标准引力参数，单位 `m^3/s^2`；
- `R`：海平面半径，单位 `m`；
- `h`：海拔，单位 `m`。

Kerbin 的常用近似值为 `R = 600000 m`、`mu ≈ 3.5316e12 m^3/s^2`。实现必须读取 `SHIP:BODY:MU` 与 `SHIP:BODY:RADIUS`，这些数值只用于解释，不能成为新的硬编码来源。

按上述近似值计算：

| 海拔 | 重力加速度 | 相对海平面 |
|---:|---:|---:|
| 0 km | 9.810 m/s² | 100.0% |
| 2 km | 9.745 m/s² | 99.34% |
| 10 km | 9.491 m/s² | 96.75% |
| 20 km | 9.187 m/s² | 93.65% |
| 30 km | 8.898 m/s² | 90.70% |
| 40 km | 8.622 m/s² | 87.89% |
| 50 km | 8.359 m/s² | 85.21% |
| 70 km | 7.867 m/s² | 80.20% |

Kerbin 的表面重力接近地球，但半径远小于地球，因此重力随海拔降低得更快。固定使用 `9.81` 会在 40 km 把重力高估约 `1.19 m/s²`，足以显著提前主制动预测。

### 3.2 地表旋转坐标系

KSP 的物理作用在惯性/世界运动上，而回收船目标和 `VELOCITY:SURFACE` 位于随 Kerbin 旋转的语义中。若预测器直接在地表旋转坐标系积分，应包含：

```text
a_surface = a_gravity + a_drag + a_thrust
            - 2 * omega × v_surface
            - omega × (omega × r)
```

Kerbin 自转角速度近似恒定，因此没有必要加入欧拉加速度项。另一种更不易出错的实现是在惯性系积分位置/速度，再在每一步把回收船和落点转换到旋转地表坐标。

## 4. Kerbin 压力、温度和密度

### 4.1 KSP 实际调用链

`CelestialBody.GetPressure(h)` 在 Kerbin 大气内计算压力，达到大气顶后返回 0。Kerbin 使用压力 `FloatCurve`，因此：

```text
P_kPa(h) = atmospherePressureCurve.Evaluate(h)
```

如果某个 Mod 天体使用归一化曲线，输入会先除以 `atmosphereDepth`；本项目必须调用游戏 API，而不是假定曲线形式。

基础温度来自温度曲线：

```text
T_base(h) = atmosphereTemperatureCurve.Evaluate(h)
```

FlightIntegrator 使用的实际大气温度还会加入太阳、纬度和时刻相关偏移：

```text
T_actual = body.GetFullTemperature(h, atmosphereTemperatureOffset)
```

密度使用理想气体关系：

```text
rho = P_kPa * 1000 * molarMass / (R_ideal * T_actual)
R_ideal = 8.31447 J/(mol*K)
```

声速和 Mach 数为：

```text
speedOfSound = sqrt(adiabaticIndex * P_kPa * 1000 / rho)
Mach         = airRelativeSpeed / speedOfSound
```

Kerbin 大气在 70 km 处被游戏截断为真空。现实世界的标准大气、固定 scale height 或地球温度分层不能直接替代这些曲线。

### 4.2 可用接口

kOS 已暴露：

- `SHIP:BODY:ATM:ALTITUDEPRESSURE(h)`；
- `SHIP:BODY:ATM:ALTITUDETEMPERATURE(h)`；
- `SHIP:BODY:ATM:HEIGHT`、`MOLARMASS`、`ADIABATICINDEX`。

其中 kOS 的高度温度是近似查询；用于精确预测的插件应直接调用 KSP `GetPressure`、`GetFullTemperature` 和 `GetDensity`，并把结果记录给遥测和预测器。

## 5. KSP stock DragCube 阻力

### 5.1 动压

FlightIntegrator 使用：

```text
q_kPa = 0.0005 * rho * V_air^2
```

这就是 `0.5 * rho * V²` 从 Pa 换成 kPa。stock Kerbin 没有天气风场时，`V_air` 与相对旋转地表的大气速度一致；计算时仍应使用 KSP 实际 part/vessel 气动速度，而不是混用轨道速度。

### 5.2 每零件阻力

当前本机 `Physics.cfg` 包含：

```text
dragMultiplier     = 8
dragCubeMultiplier = 0.1
```

对使用 DragCube 的单个零件，FlightIntegrator 的力可整理为：

```text
D_part_kN = q_kPa
             * DragCubes.AreaDrag(Mach, localFlowDirection, occlusion)
             * dragCubeMultiplier
             * DragCurvePseudoReynolds(rho * V_air)
             * dragMultiplier
```

方向与气动速度相反，力作用在零件阻力中心，因此除减速外还会产生姿态力矩。整箭阻力是所有未被气流屏蔽零件的结果之和。

### 5.3 `AreaDrag` 不是固定面积

每个 DragCube 有六个方向面的面积和阻力值。游戏每个物理帧根据以下因素重算：

- 气流在零件局部坐标系中的方向；
- 六个面与气流的点积；
- tip、surface、tail 的 Mach 曲线；
- `DRAG_MULTIPLIER` 跨音速/超音速曲线；
- DragCube 原始阻力值经过 `DRAG_CD` 和 `DRAG_CD_POWER` 的变换；
- 相邻零件遮挡后的有效面积；
- 可变构型零件当前 DragCube 权重。

`DragCurvePseudoReynolds` 的输入只是 `rho * V`，并不包含真实雷诺数所需的特征长度和动力黏度。因此原版 KSP 气动是为游戏构造的近似，不能直接套用现实火箭的固定 `Cd`。

另外，长箭体偏离速度方向时还会产生 body lift、lifting-surface lift 和角阻尼；30°姿态约束不仅保护箭体，也显著限制了 DragCube 等效迎风面积的变化范围。

## 6. 当前项目模型审计

### 6.1 实机 kOS 脚本

`Ships/Script/cz10b/main.ks` 已使用：

```text
SHIP:BODY:MU / (SHIP:BODY:RADIUS + SHIP:ALTITUDE)^2
```

局部重力大小的处理正确。

无动力段通过表面速度差估计总加速度，然后减去重力并低通得到垂直/水平“阻力”估计。这种方法有三个限制：

1. 它只能在已经飞过的状态上测量，不能独立预测未经历的 Mach/迎角区间；
2. 若直接对旋转地表速度向量作差，科里奥利、离心项和坐标方向变化可能被误吸收到“阻力”中；
3. 只保存垂直标量和水平向量，无法表达 DragCube 随 Mach、姿态和构型变化的完整关系。

### 6.2 离线点质量模型

`tools/simulate_controller.py` 明确声明它不是 KSP 高保真大气/推进仿真。目前使用：

```text
G = 9.81
density_ratio = exp(-h / 5000)
vertical_drag   = -2.5e-5 * density_ratio * vv * abs(vv)
horizontal_drag = -3.0e-5 * density_ratio * vh * abs(vh)
```

这些量是为控制器回归调出的经验项，不能用于宣称 40 km 入口轨迹或75%主制动点在 KSP 中精确可达。它仍可保留用于快速发现 PWM、反弹、符号错误和明显不可达状态。

## 7. 已确认采用的混合模型

### 7.1 状态和积分

预测器至少积分以下状态：

```text
position, inertialVelocity, mass, missionPhase
```

每一步计算：

```text
a = gravity(mu, position)
  + drag(P, T, rho, Mach, attitude, vehicleConfiguration)
  + commandedThrust(actualAvailableThrust, mass, direction)
```

推荐在惯性系使用固定小步长 Heun/RK4 积分，再把预测位置转换为回收船地表坐标。步长和计算预算应通过实机帧率验证，不能让预测器拖慢物理仿真或改变控制时序。

### 7.2 游戏环境基础层

插件逐物理帧提供或记录：

- `mu`、`R`、重力矢量和重力大小；
- 海拔、压力、完整温度、密度、声速、Mach 和动压；
- Kerbin 自转角速度；
- 一级质量、实际可用推力、实际推力和推进剂比例；
- 每个零件或整箭合计的实际 `dragScalar`、body lift 和气动力矩；
- 当前 DragCube `AreaDrag`、气动速度方向和遮挡状态摘要。

### 7.3 等效气动表

以 KSP 的当前构型数据为基础建立：

```text
CDA_effective = totalDrag_kN / q_kPa
```

并按至少以下变量索引：

```text
Mach, angleOfAttack, stageConfiguration
```

质量不属于 `CDA` 本身，但必须在换算阻力加速度时使用当前质量。对于30°以内的回收姿态，可重点覆盖实际允许的迎角区域，不需要为不合格的大姿态建立控制权限。

### 7.4 无动力在线校正

在发动机实际推力为零且载具未碰撞/捕获时，用惯性速度差计算实测非重力加速度：

```text
a_drag_measured = dv_inertial/dt - a_gravity
```

若实现选择地表旋转系，则必须显式扣除科里奥利和离心项。用测得值更新：

- 当前 Mach/迎角格点的 `CDA_effective`；
- 预测阻力与实测阻力的比例残差；
- 模型不确定度上界。

发动机点火、分级、钢索接触或明显姿态瞬变期间不得把总加速度误认成空气阻力。

### 7.5 在任务状态机中的用途

1. **40 km入口减速**：连续预测关机后的无动力落点；燃烧目标仍是水平地速 `≤1000 m/s`，同时让预测轨迹指向回收船。
2. **无动力检查点**：用更新后的混合模型计算预测落点偏差；只有超过预先声明阈值才执行一次连续修正。
3. **75%主制动点**：搜索最晚可行点火时刻，使连续75%标称推力在海拔2 km达到下降速度150～200 m/s、水平地速≤5 m/s、水平误差≤10 m。
4. **在线重规划**：只更新模型参数和剩余轨迹，不允许把主制动变成0/75% PWM，也不能靠反复延后终点制造悬停。

### 7.6 模型与控制的边界

混合模型负责预测重力、阻力、落点和主制动时机；制导算法负责在30°喷管/速度锥、75%标称推力和连续油门约束下跟踪轨迹。GFOLD、PID、解析走廊或其他控制方法仍可组合使用，但都必须通过强制验收基线。

## 8. 遥测与校准记录

每次用于模型校准的实机任务至少记录：

```text
UT, phase, altitude
position/velocity in inertial and surface frames
mu, gravity magnitude/vector, body angular velocity
pressure, temperature, density, speed of sound, Mach, dynamic pressure
mass, propellant fraction, commanded/actual throttle, actual thrust
attitude, angle of attack, nozzle/velocity angle
DragCube AreaDrag, total dragScalar, body lift
predicted drag acceleration, measured drag acceleration, residual
predicted impact point/error, predicted 2 km state
```

需要同时记录计算使用的模型版本标识和参数快照，以便比较不同实飞；该标识是遥测可追溯信息，不是任务通过条件，也不是 Git 提交要求。

## 9. 明确禁止的替代做法

- 不得使用现实地球标准大气直接代替 Kerbin 曲线。
- 不得用固定 `9.81 m/s²`规划40 km至2 km轨迹。
- 不得用单一指数密度和固定阻力系数作为最终实机结论。
- 不得用离线点质量模型的成功替代 KSP 完整任务。
- 不得在模型误差较大时用低空长悬停、PWM或超过30°姿态补救。

## 10. 资料和本机依据

- kOS `Body:MU`、`Body:RADIUS`：<https://ksp-kos.github.io/KOS_DOC/structures/celestial_bodies/body.html>
- kOS 大气压力、温度和参数接口：<https://ksp-kos.github.io/KOS_DOC/structures/celestial_bodies/atmosphere.html>
- 本机 KSP 1.12.5：`KSP_x64_Data/Managed/Assembly-CSharp.dll` 中的 `CelestialBody`、`FlightIntegrator`、`DragCubeList`、`PhysicsGlobals`。
- 本机原版气动曲线和全局系数：`C:/Projects/Kerbal Space Program/Physics.cfg`。
- 当前实机控制器：`Ships/Script/cz10b/main.ks`。
- 当前离线回归：`tools/simulate_controller.py`。
