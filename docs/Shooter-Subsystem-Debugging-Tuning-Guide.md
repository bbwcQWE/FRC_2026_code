# Shooter子系统调试与调优实战指南

## 一、子系统架构概览

### 1.1 组件构成

Shooter子系统由三个核心执行机构组成，采用分层控制架构：

| 组件 | 控制器 | 电机型号 | CAN ID | 控制模式 |
|------|--------|----------|--------|----------|
| FlyWheel | TalonFX | Kraken X60 | 17 (主) / 18 (从) | Closed Loop Velocity |
| Hood | SparkMax | NEO 550 | 16 | Closed Loop Position |
| Turret | TalonFX | Kraken X60 | 19 | Closed Loop Position |

### 1.2 代码结构

```
ShooterSubsystem (父类，状态协调)
├── FlyWheelSubsystem (飞轮电机控制)
├── HoodSubsystem (瞄准机构控制)
├── TurretSubsystem (炮塔旋转控制)
└── SOTMCalculator (移动中射击计算)
```

### 1.3 核心控制逻辑

- **状态机**: isReady / isAiming / isShooting / isSOTMEnabled
- **查表机制**: 基于距离的LUT查找 (hoodAngle, RPM, timeOfFlight)
- **SOTM**: 向量减法实现移动中射击 (V_shot = V_target - V_robot)

---

## 二、FlyWheel (飞轮) 调试指南

### 2.1 硬件配置参数

**电机配置** (FlyWheelSubsystem.java:49-63):

| 参数 | 当前值 | 说明 | 调优建议 |
|------|--------|------|----------|
| kP | 50 | 比例增益 | 初始值，可根据响应调整 |
| kI | 0 | 积分增益 | 保持为0 |
| kD | 0 | 微分增益 | 保持为0 |
| kV | 0 | 速度前馈 | 当前未使用feedforward |
| kA | 0 | 加速度前馈 | 当前未使用feedforward |
| Gear Ratio | 1:1 | 齿轮比 | 直接驱动无需减速 |
| Current Limit | 40A | 电流限制 | 防止电机损坏 |
| Idle Mode | COAST | 空闲模式 | 发射后惯性滑行 |

### 2.2 调试步骤

#### 步骤1: 基本转动测试
```java
// 在RobotContainer中添加测试命令
Command testFlywheel = flywheel.setVelocity(RPM.of(3000));
```

**预期行为**:
- 电机平滑加速到3000 RPM
- 无异常噪音
- 电流控制在40A以内

#### 步骤2: 速度响应测试
使用Shuffleboard/Glass监控:
- `Flywheel/Velocity` (实际速度)
- `Flywheel/Setpoint` (目标速度)
- `Flywheel/Voltage` (输出电压)

**观察指标**:
- 上升时间 (rise time): < 0.5s 为宜
- 超调量 (overshoot): < 5% 为宜
- 稳态误差 (steady-state error): < 50 RPM

#### 步骤3: PID调优

**调优方法 - Ziegler-Nichols:**
1. 将kP从较小值(如1)逐渐增大
2. 找到产生持续振荡的临界增益kCu
3. 记录振荡周期Pu
4. 计算最终增益: kP = 0.45*kCu, kI = 0.54*kCu/Pu

**本项目推荐初始值**:
```java
.withClosedLoopController(25, 0, 0, DegreesPerSecond.of(90), DegreesPerSecond.of(45))
```

### 2.3 Feedforward调优

**物理意义**:
- kS (静态): 克服摩擦力
- kV (速度): 产生期望速度所需的电压
- kA (加速度): 克服惯性

**调优方法**:
```java
// 测量方法: 施加不同速度，记录所需电压
// 公式: Voltage = kV * Velocity + kA * Acceleration + kS

.withFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
```

**典型值参考**:
- kS: 0.1 - 0.5V
- kV: 0.05 - 0.12 V/(deg/s)
- kA: 0.01 - 0.05 V/(deg/s²)

### 2.4 常见故障排查

| 故障现象 | 可能原因 | 排查方法 |
|----------|----------|----------|
| 电机不转 | CAN连线问题 | 检查Phoenix Tuner连接 |
| 速度波动大 | PID增益过高 | 降低kP，检查反馈信号 |
| 启动延迟 | 摩擦力过大 | 增加kS值 |
| 电流持续高 | 机械卡滞 | 检查轴承、齿轮 |

---

## 三、Hood (瞄准机构) 调试指南

### 3.1 硬件配置参数

**电机配置** (HoodSubsystem.java:51-64):

| 参数 | 当前值 | 说明 | 调优建议 |
|------|--------|------|----------|
| kP | 0.00016541 | 位置比例增益 | 极小值，需精确调校 |
| kI | 0 | 积分增益 | 保持为0 |
| kD | 0 | 微分增益 | 保持为0 |
| kV | 0.089836 V/(rot/s) | 速度前馈 | 实际feedforward参数 |
| kA | 0.014557 V/(rot/s²) | 加速度前馈 | 实际feedforward参数 |
| Gear Ratio | 3:4 | 齿轮减速 | 降低转速增加扭矩 |
| Current Limit | 40A | 电流限制 | 防止烧毁电机 |
| Soft Limits | 5° - 100° | 软件限位 | 机械保护 |
| Hard Limits | 0° - 120° | 硬件限位 | 物理极限 |

### 3.2 Feedforward参数

**当前配置** (第62行):
```java
.withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
```

**物理意义**:
- kS = 0.27937V: 克服重力矩
- kV = 0.089836 V/(rot/s): 速度相关电压
- kA = 0.014557 V/(rot/s²): 加速度相关电压

**调优提示**:
此参数通过SysId实验获得，保持当前值即可。如出现抖动，适当增大kS。

### 3.3 调试步骤

#### 步骤1: 机械校准
```java
// 检查软限位是否正确
// 观察HoodInputs.position是否在合理范围(5°-100°)
```

#### 步骤2: 位置响应测试
```java
// 测试命令
Command testHood = hood.setAngle(Degrees.of(45));
```

**监控指标**:
- `Hood/Setpoint` vs `Hood/Position`
- 响应时间 < 1s
- 无振荡

#### 步骤3: 角度容差调整

当前容差设置 (ShooterSubsystem.java:64):
```java
public static final double HOOD_ANGLE_TOLERANCE_DEG = 1.5; // 度
```

**调整建议**:
- 过抛: 减小容差 (如1.0°)
- 不稳定: 增大容差 (如2.0°)

### 3.4 常见故障排查

| 故障现象 | 可能原因 | 排查方法 |
|----------|----------|----------|
| 角度偏差大 | 编码器零位偏移 | 检查initialize位置设置 |
| 运动不流畅 | 齿轮间隙 | 检查齿轮装配 |
| 撞到限位 | 软限位失效 | 验证限位开关 |
| 电机过热 | 连续工作过长 | 降低工作频率 |

---

## 四、Turret (炮塔) 调试指南

### 4.1 硬件配置参数

**电机配置** (TurretSubsystem.java:56-67):

| 参数 | 当前值 | 说明 | 调优建议 |
|------|--------|------|----------|
| kP | 4 | 位置比例增益 | 需要精确控制 |
| kI | 0 | 积分增益 | 保持为0 |
| kD | 0 | 微分增益 | 保持为0 |
| kV | 180 deg/s | 速度前馈 | 中速响应 |
| kA | 90 deg/s² | 加速度前馈 | 平滑启停 |
| Gear Ratio | 3:4 | 齿轮减速 | 大减速比增加扭矩 |
| Current Limit | 40A | 电流限制 | 防止卡死烧毁 |
| Hard Limits | 0° - 720° | 硬件限位 | 两圈行程 |

### 4.2 CRT (Canary Ring Technology) 校准

**CRT计算器** (TurretCRTCalculator.java):
- 用于绝对位置初始化
- 在构造函数中自动调用initializeFromCRT()

**重同步方法**:
```java
// 命令触发
Command resyncCRT = turret.resyncCRT();
```

**CRT状态监控**:
- `Turret/CRTInitialized`: 初始化状态
- `Turret/CRTResync`: 重同步状态
- `Turret/crtStatus`: 详细状态字符串
- `Turret/crtAngle`: CRT计算角度

### 4.3 调试步骤

#### 步骤1: CRT初始化验证
检查日志输出:
```
[TurretCRT] INFO: CRT initialized with angle: X degrees
```

#### 步骤2: 旋转测试
```java
// 测试命令 - 左右旋转
Command testTurretLeft = turret.setAngle(Degrees.of(-30));
Command testTurretRight = turret.setAngle(Degrees.of(30));
```

#### 步骤3: 连续旋转测试
验证720°行程无卡滞:
```java
Command testFullRotation = turret.setAngle(Degrees.of(360));
```

### 4.4 常见故障排查

| 故障现象 | 可能原因 | 排查方法 |
|----------|----------|----------|
| 角度跳变 | CRT初始化失败 | 检查CAN线连接 |
| 旋转异响 | 齿轮磨损 | 检查齿轮状态 |
| 位置漂移 | 编码器干扰 | 添加屏蔽 |
| 电机过热 | 摩擦力过大 | 润滑轴承 |

---

## 五、SOTM (移动中射击) 调试指南

### 5.1 LUT查找表配置

**数据位置** (SOTMCalculator.java:143-152):

| 距离(m) | Hood角度(°) | RPM | 飞行时间(s) |
|---------|-------------|-----|-------------|
| 1.5 | 65.0 | 2000 | 0.40 |
| 2.0 | 55.0 | 3000 | 0.50 |
| 2.5 | 50.0 | 3500 | 0.55 |
| 3.0 | 45.0 | 4000 | 0.60 |
| 3.5 | 42.0 | 4400 | 0.66 |
| 4.0 | 38.0 | 4800 | 0.72 |
| 4.5 | 35.0 | 5100 | 0.78 |
| 5.0 | 32.0 | 5200 | 0.85 |
| 5.5 | 30.0 | 5400 | 0.92 |
| 6.0 | 28.0 | 5500 | 0.98 |

### 5.2 LUT调优步骤

#### 步骤1: 静态射击测试
在固定位置测试，记录:
- 实际落点与目标偏差
- 修正LUT数据

#### 步骤2: 更新LUT
```java
// 使用SOTMCalculator的方法更新
sotmCalculator.updateLUT(distance, hoodAngle, rpm, timeOfFlight);
```

#### 步骤3: SOTM验证
移动中射击测试:
- 测试不同速度 (1-3 m/s)
- 测试不同方向 (0°, 45°, 90°)

### 5.3 延迟补偿参数

**当前配置** (SOTMCalculator.java:66):
```java
private double latencyCompensation = 0.10; // 100ms
```

**调整方法**:
- 增加延迟: 补偿视觉+控制延迟
- 减少延迟: 更精确的预判

### 5.4 调试监控指标

**AdvantageKit日志**:
- `SOTM/TurretAngle`: 炮塔角度
- `SOTM/Distance`: 目标距离
- `SOTM/RequiredVelocity`: 所需速度
- `SOTM/EffectiveDistance`: 有效距离
- `SOTM/IsValid`: 有效性标志

---

## 六、参数快速参考表

### 6.1 ShooterSubsystem常量

```java
// ShooterSubsystem.java
private static final double HOOD_MIN_ANGLE = 20.0;   // Hood最小角度
private static final double HOOD_MAX_ANGLE = 70.0;   // Hood最大角度
private static final double FLYWHEEL_MIN_RPM = 1000; // 飞轮最小转速
private static final double FLYWHEEL_MAX_RPM = 6000; // 飞轮最大转速

// 容差参数
public static final double VELOCITY_TOLERANCE_RPM = 200.0;   // 速度容差
public static final double HOOD_ANGLE_TOLERANCE_DEG = 1.5;   // 角度容差
```

### 6.2 调试命令速查

| 测试项 | 命令 | 预期结果 |
|--------|------|----------|
| 飞轮速度 | `flywheel.setVelocity(RPM.of(3000))` | 平滑加速 |
| Hood位置 | `hood.setAngle(Degrees.of(45))` | 精确到位 |
| 炮塔旋转 | `turret.setAngle(Degrees.of(90))` | 360°无卡滞 |
| 完整发射序列 | `shooter.aimAtTarget(3.0, 0).andThen(prepareToShoot())` | 自动流程 |
| CRT重同步 | `turret.resyncCRT()` | 角度复位 |
| SysId飞轮 | `flywheel.sysId()` | PID测量 |
| SysId Hood | `hood.sysId()` | PID测量 |

---

## 七、WPILib工具链使用

### 7.1 Shuffleboard/Glass监控

**推荐布局**:
```
[Flywheel/Velocity] [Flywheel/Setpoint] [Flywheel/Error]
[Hood/Position]     [Hood/Setpoint]     [Hood/Error]
[Turret/Position]  [Turret/Setpoint]    [Turret/CRTStatus]
[SOTM/Distance]    [SOTM/TurretAngle]  [SOTM/IsValid]
```

### 7.2 DataLog分析

使用 AdvantageScope 分析:
- 加载 .wpilog 文件
- 时间轴同步多个信号
- 事件标记 (DriverStation events)

### 7.3 SysId自动化测试

```bash
# 运行SysId
./gradlew simulateJava
# 在Shuffleboard中选择SysId Routine
# 按步骤完成测试
```

---

## 八、代码架构改进建议

### 8.1 潜在改进点

1. **Feedforward参数集中管理**
   - 当前分散在各个Subsystem中
   - 建议提取到Constants类

2. **LUT数据外部化**
   - 当前硬编码在SOTMCalculator中
   - 建议移至配置文件

3. **状态机规范化**
   - 建议使用enum管理状态转换

### 8.2 调优优先级

| 优先级 | 项目 | 影响 |
|--------|------|------|
| P0 | FlyWheel PID | 射击一致性 |
| P0 | Hood位置精度 | 落点准确性 |
| P1 | SOTM LUT | 移动中射击 |
| P2 | Turret响应速度 | 瞄准速度 |

---

*本文档基于11319_2026_Alphabot_code代码库分析生成*
*最后更新: 2026-03-20*
