# Intake子系统调试与调优实战指南

## 一、子系统架构概览

### 1.1 组件构成

Intake子系统采用双电机控制设计：

| 组件 | 控制器 | 电机型号 | CAN ID | 控制模式 |
|------|--------|----------|--------|----------|
| Turn Motor | SparkMax | NEO | 25 | Closed Loop Position |
| Drive Motor | TalonFX | Kraken X60 | 26 | Closed Loop Velocity |

### 1.2 机构描述

- **Turn (翻转机构)**: 控制Intake的收放角度，0°为收起状态，90°为展开状态
- **Drive (驱动机构)**: 使用FlyWheel原理驱动辊子，实现球/件的摄入

### 1.3 代码结构

```
IntakeSubsystem
├── Arm turn (翻转机构 - 基于yams Arm)
└── FlyWheel drive (驱动机构 - 基于yams FlyWheel)
```

---

## 二、Turn Motor (翻转机构) 调试指南

### 2.1 硬件配置参数

**电机配置** (IntakeSubsystem.java:60-75):

| 参数 | 当前值 | 说明 | 调优建议 |
|------|--------|------|----------|
| kP | 4 | 位置比例增益 | 响应速度快 |
| kI | 0 | 积分增益 | 保持为0 |
| kD | 0 | 微分增益 | 保持为0 |
| kV | 0 | 速度前馈 | 当前未使用feedforward |
| kA | 0 | 加速度前馈 | 当前未使用feedforward |
| Gear Ratio | 1:1 | 齿轮比 | 直接驱动 |
| Current Limit | 30A | 电流限制 | 适中 |
| Idle Mode | BRAKE | 空闲制动 | 保持收起位置 |
| Soft Limits | 0° - 90° | 软件限位 | 防止过度旋转 |
| Hard Limits | -10° - 100° | 硬件限位 | 安全边界 |

### 2.2 机构参数

**Arm配置** (IntakeSubsystem.java:80-87):

| 参数 | 当前值 | 说明 |
|------|--------|------|
| Starting Position | 0° | 初始收起状态 |
| Soft Limits | 0° - 90° | 工作范围 |
| Hard Limits | -10° - 100° | 物理极限 |
| Length | 0.3m | 臂长 |
| Mass | 2kg | 质量 |

### 2.3 调试步骤

#### 步骤1: 机械限位验证

**检查软限位是否正常工作**:
```java
// 监控日志
Intake/TurnSetpoint  // 目标位置
Intake/turnPosition  // 实际位置
```

**验证边界**:
- 收起位置 (0°): 机构完全收回
- 展开位置 (90°): 机构完全展开

#### 步骤2: 翻转响应测试

```java
// 测试命令
Command deployIntake = intake.deploy();   // 展开到90°
Command retractIntake = intake.retract(); // 收起到0°
```

**观察指标**:
- 翻转时间: < 0.5s 为宜
- 无异响
- 电流峰值 < 30A

#### 步骤3: 位置精度调优

如果出现位置偏差:
- 调整kP值 (范围2-8)
- 检查编码器零位
- 验证软限位触发

### 2.4 常见故障排查

| 故障现象 | 可能原因 | 排查方法 |
|----------|----------|----------|
| 翻转不到位 | 限位开关触发过早 | 调整软限位值 |
| 机构抖动 | PID增益过高 | 降低kP |
| 异响 | 齿轮磨损 | 检查齿轮状态 |
| 电流过大 | 机械卡滞 | 检查转动阻力 |

---

## 三、Drive Motor (驱动机构) 调试指南

### 3.1 硬件配置参数

**电机配置** (IntakeSubsystem.java:93-105):

| 参数 | 当前值 | 说明 | 调优建议 |
|------|--------|------|----------|
| kP | 50 | 速度比例增益 | 高速响应 |
| kI | 0 | 积分增益 | 保持为0 |
| kD | 0 | 微分增益 | 保持为0 |
| kV | 0 | 速度前馈 | 当前未使用feedforward |
| kA | 0 | 加速度前馈 | 当前未使用feedforward |
| Gear Ratio | 1:1 | 齿轮比 | 直接驱动 |
| Current Limit | 30A | 电流限制 | 适中 |
| Idle Mode | BRAKE | 空闲制动 | 停止时保持 |

### 3.2 辊子参数

**FlyWheel配置** (IntakeSubsystem.java:110-111):

| 参数 | 当前值 | 说明 |
|------|--------|------|
| Diameter | 0.0762m (3英寸) | 辊子直径 |
| Mass | 0.5kg | 辊子质量 |

### 3.3 调试步骤

#### 步骤1: 辊子转动测试

```java
// 测试命令
Command testIntake = intake.setDriveVelocity(RPM.of(500));
```

**预期行为**:
- 辊子平滑转动
- 方向正确 (向内 intake)

#### 步骤2: 转速响应测试

监控信号:
- `Intake/DriveSetpoint` (目标转速)
- `Intake/driveVelocity` (实际转速)
- `Intake/driveVoltage` (输出电压)
- `Intake/driveCurrent` (电机电流)

**观察指标**:
- 响应时间 < 0.3s
- 稳态误差 < 100 RPM

#### 步骤3:  Intake/Eject功能测试

```java
// 吸取测试
Command intakeNote = intake.intake();  // deploy + 500 RPM

// 吐出测试
Command ejectNote = intake.eject();    // -500 RPM
```

### 3.4 Intake速度优化

**基于游戏策略的速度选择**:

| 应用场景 | 推荐速度 | 说明 |
|----------|----------|------|
| 场地 intake | 300-500 RPM | 中速稳定 |
| 快速 intake | 600-800 RPM | 高速有风险 |
| 抛出对手部件 | -300 RPM | 反向旋转 |

### 3.5 常见故障排查

| 故障现象 | 可能原因 | 排查方法 |
|----------|----------|----------|
| 不转动 | CAN连线问题 | 检查Phoenix Tuner |
| 速度慢 | 增益过低 | 提高kP |
| 球/件卡住 | 方向错误 | 确认旋转方向 |
| 电流持续高 | 机械阻力大 | 检查轴承 |

---

## 四、系统集成调试

### 4.1 完整工作流程

**自动 intake 序列**:
```java
// 典型命令序列
intake.deploy()           // 展开机构
    .andThen(intake.setDriveVelocity(RPM.of(500))) // 开始驱动
```

**安全停止序列**:
```java
intake.stop();  // 停止所有运动，保持当前位置
```

### 4.2 与其他子系统协同

#### 4.2.1 与Feeder协同

典型场景: Intake → Feeder → Shooter

```java
// 需要保证时序正确
intake.intake()          // 展开并驱动
    .alongWith(feeder.runFeeder())  // 同时启动feeder
```

#### 4.2.2 与Shooter协同

避免在Shooting时误触发intake:
- 使用Trigger条件判断
- 添加互锁逻辑

### 4.3 状态监控

**AdvantageKit日志**:

| 信号 | 说明 |
|------|------|
| Intake/TurnSetpoint | 翻转目标位置 |
| Intake/turnPosition | 翻转实际位置 |
| Intake/DriveSetpoint | 驱动目标转速 |
| Intake/driveVelocity | 驱动实际转速 |
| Intake/turnVoltage | 翻转电机电压 |
| Intake/driveVoltage | 驱动电机电压 |
| Intake/turnCurrent | 翻转电机电流 |
| Intake/driveCurrent | 驱动电机电流 |

---

## 五、参数快速参考

### 5.1 默认速度常量

| 命令 | 速度值 | 用途 |
|------|--------|------|
| intake() | 500 RPM | 标准吸取 |
| eject() | -500 RPM | 吐出/抛出 |
| stop() | 0 RPM | 停止 |

### 5.2 角度常量

| 位置 | 角度值 |
|------|--------|
| 收起位置 | 0° |
| 展开位置 | 90° |

### 5.3 限位参数

| 类型 | 最小值 | 最大值 |
|------|--------|--------|
| Soft Limits | 0° | 90° |
| Hard Limits | -10° | 100° |

---

## 六、调试命令速查

| 测试项 | 命令 | 预期结果 |
|--------|------|----------|
| 展开机构 | `intake.deploy()` | 翻转至90° |
| 收起机构 | `intake.retract()` | 翻转至0° |
| 驱动测试 | `intake.setDriveVelocity(RPM.of(500))` | 辊子转动 |
| 吸取测试 | `intake.intake()` | 展开+驱动 |
| 吐出测试 | `intake.eject()` | 反向驱动 |
| 完全停止 | `intake.stop()` | 翻转+驱动停止 |

---

## 七、WPILib工具链使用

### 7.1 Shuffleboard/Glass布局建议

```
[Intake/turnPosition] [Intake/TurnSetpoint]
[Intake/driveVelocity] [Intake/DriveSetpoint]
[Intake/turnCurrent]   [Intake/driveCurrent]
```

### 7.2 实时调整技巧

使用Shuffleboard的Manual mode:
1. 找到对应的Sendable
2. 直接拖动滑块调整
3. 观察实时响应
4. 记录最佳值

---

## 八、安全注意事项

### 8.1 机械安全

1. **翻转限位必须工作**: 防止机构过转损坏
2. **电流限制必须设置**: 防止电机烧毁
3. **定期检查齿轮**: 防止磨损断裂

### 8.2 操作安全

1. **赛前检查**: 验证限位功能
2. **赛中监控**: 观察电流异常
3. **赛后维护**: 清洁检查机构

---

## 九、代码改进建议

### 9.1 建议改进

1. **速度曲线**: 当前为恒定速度，建议增加斜坡
2. **超时保护**: 添加超时自动停止
3. **状态反馈**: 添加到位检测

### 9.2 调优优先级

| 优先级 | 项目 | 影响 |
|--------|------|------|
| P0 | 翻转限位 | 机械安全 |
| P1 | 驱动响应 | intake可靠性 |
| P2 | 速度优化 | 效率提升 |

---

*本文档基于代码库分析生成*
*最后更新: 2026-03-20*
