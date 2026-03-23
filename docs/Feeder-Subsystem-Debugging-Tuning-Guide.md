# Feeder子系统调试与调优实战指南

## 一、子系统架构概览

### 1.1 组件构成

Feeder子系统包含两个独立的驱动机构：

| 组件 | 控制器 | 电机型号 | CAN ID | 控制模式 |
|------|--------|----------|--------|----------|
| Washing Machine | SparkMax | NEO | 21 | Closed Loop Velocity |
| Indexer | TalonFX | Kraken X60 | 20 | Closed Loop Velocity |

### 1.2 机构描述

- **Washing Machine (清洗机/供料轮)**: 主要供料机构，使用类似"洗衣机"的辊子设计
- **Indexer (分度器/加速轮)**: 二级供料，将球/件加速送入Shooter

### 1.3 代码结构

```
FeederSubsystem
├── FlyWheel washingMachine (供料轮)
└── FlyWheel indexer (分度器)
```

---

## 二、Washing Machine (供料轮) 调试指南

### 2.1 硬件配置参数

**电机配置** (FeederSubsystem.java:51-63):

| 参数 | 当前值 | 说明 | 调优建议 |
|------|--------|------|----------|
| kP | 50 | 速度比例增益 | 高速响应 |
| kI | 0 | 积分增益 | 保持为0 |
| kD | 0 | 微分增益 | 保持为0 |
| kV | 0 | 速度前馈 | 当前未使用feedforward |
| kA | 0 | 加速度前馈 | 当前未使用feedforward |
| Gear Ratio | 20:1 | 大减速比 | 增加扭矩 |
| Current Limit | 30A | 电流限制 | 适中 |
| Idle Mode | COAST | 空闲模式 | 惯性滑行 |

### 2.2 辊子参数

**FlyWheel配置** (FeederSubsystem.java:68-73):

| 参数 | 当前值 | 说明 |
|------|--------|------|
| Diameter | 0.0762m (3英寸) | 辊子直径 |
| Mass | 0.454kg (1磅) | 辊子质量 |

### 2.3 调试步骤

#### 步骤1: 基础转动测试

```java
// 测试命令
Command testWashingMachine = feeder.setWashingMachineVelocity(RPM.of(500));
```

**观察指标**:
- 电机平滑转动
- 电流 < 30A
- 无异常噪音

#### 步骤2: 速度响应测试

监控信号:
- `Feeder/WashingMachineSetpoint` (目标转速)
- `Feeder/washingMachineVelocity` (实际转速)
- `Feeder/washingMachineVoltage` (输出电压)
- `Feeder/washingMachineCurrent` (电机电流)

#### 步骤3: 供料测试

```java
// 测试供料功能
Command feedNote = feeder.setWashingMachineVelocity(RPM.of(500));
```

**验证**:
- 球/件能被正确夹持
- 供料速度适中
- 无卡滞

### 2.4 常见故障排查

| 故障现象 | 可能原因 | 排查方法 |
|----------|----------|----------|
| 不转动 | CAN/供电问题 | 检查连接 |
| 速度不足 | 增益偏低 | 提高kP |
| 球/件打滑 | 速度过高 | 降低转速 |
| 电流过高 | 机械阻力大 | 检查轴承 |

---

## 三、Indexer (分度器) 调试指南

### 3.1 硬件配置参数

**电机配置** (FeederSubsystem.java:77-89):

| 参数 | 当前值 | 说明 | 调优建议 |
|------|--------|------|----------|
| kP | 50 | 速度比例增益 | 高速响应 |
| kI | 0 | 积分增益 | 保持为0 |
| kD | 0 | 微分增益 | 保持为0 |
| kV | 0 | 速度前馈 | 当前未使用feedforward |
| kA | 0 | 加速度前馈 | 当前未使用feedforward |
| Gear Ratio | 2:1 | 减速比 | 中等扭矩 |
| Current Limit | 30A | 电流限制 | 适中 |
| Idle Mode | BRAKE | 空闲制动 | 保持停止 |

### 3.2 辊子参数

**FlyWheel配置** (FeederSubsystem.java:94-99):

| 参数 | 当前值 | 说明 |
|------|--------|------|
| Diameter | 0.0508m (2英寸) | 较小直径 |
| Mass | 0.227kg (0.5磅) | 轻量设计 |

### 3.3 调试步骤

#### 步骤1: 基础转动测试

```java
// 测试命令
Command testIndexer = feeder.setIndexerVelocity(RPM.of(500));
```

#### 步骤2: 加速功能测试

```java
// 测试分度功能 - 比washing machine更快
Command indexNote = feeder.setIndexerVelocity(RPM.of(800));
```

### 3.4 常见故障排查

| 故障现象 | 可能原因 | 排查方法 |
|----------|----------|----------|
| 不转动 | CAN连接 | 检查 |
| 转速波动 | PID参数 | 调整kP |
| 球/件堆积 | 速度不匹配 | 调整两辊速比 |

---

## 四、系统集成调试

### 4.1 协同控制

#### 4.1.1 两辊速度匹配

**核心原则**: Indexer速度 > Washing Machine速度

这确保球/件:
- 不会在两辊之间堆积
- 加速送入Shooter

**推荐速比**:

| 场景 | Washing Machine | Indexer | 效果 |
|------|-----------------|---------|------|
| 标准供料 | 500 RPM | 600 RPM | 稳定 |
| 快速供料 | 700 RPM | 900 RPM | 高速 |
| 保守供料 | 400 RPM | 500 RPM | 低速 |

#### 4.1.2 完整供料序列

```java
// 典型供料命令
Command runFeeder = feeder.runFeeder(RPM.of(500), RPM.of(600));
Command stopFeeder = feeder.stopFeeder();
```

### 4.2 与其他子系统协同

#### 4.2.1 与Shooter协同

确保Shooter准备好后再启动Feeder:
```java
// 建议使用条件Trigger
// 当 isReadyToShoot() 为true时允许feeder
```

#### 4.2.2 与Intake协同

典型序列:
```java
intake.intake()           // intake展开并驱动
    .alongWith(feeder.runFeeder())  // 同时启动feeder
```

### 4.3 状态监控

**AdvantageKit日志**:

| 信号 | 说明 |
|------|------|
| Feeder/WashingMachineSetpoint | 供料轮目标转速 |
| Feeder/WashingMachineVelocity | 供料轮实际转速 |
| Feeder/IndexerSetpoint | 分度器目标转速 |
| Feeder/IndexerVelocity | 分度器实际转速 |
| Feeder/WashingMachineVoltage | 供料轮电压 |
| Feeder/IndexerVoltage | 分度器电压 |
| Feeder/WashingMachineCurrent | 供料轮电流 |
| Feeder/IndexerCurrent | 分度器电流 |

---

## 五、参数快速参考

### 5.1 速度命令

```java
// 默认速度
feeder.runFeeder()  // 500 RPM + 500 RPM

// 指定速度
feeder.runFeeder(RPM.of(washingMachineSpeed), RPM.of(indexerSpeed))

// 停止
feeder.stopFeeder()
```

### 5.2 调试命令速查

| 测试项 | 命令 | 预期结果 |
|--------|------|----------|
| 供料轮测试 | `feeder.setWashingMachineVelocity(RPM.of(500))` | 转动 |
| 分度器测试 | `feeder.setIndexerVelocity(RPM.of(500))` | 转动 |
| 完整测试 | `feeder.runFeeder()` | 两辊同时转动 |
| 停止测试 | `feeder.stopFeeder()` | 两辊停止 |

---

## 六、调优指南

### 6.1 速度优化

#### 影响因素:

1. **Shooter接收速度**: 必须与Shooter吞吐能力匹配
2. **球/件类型**: 不同尺寸/重量需要不同速度
3. **机械效率**: 辊子表面摩擦力

#### 调整策略:

| 现象 | 调整 |
|------|------|
| 球/件堆积 | 提高Indexer速度 |
| 球/件打滑 | 提高Washing Machine速度 |
| shooter卡顿 | 降低整体速度 |

### 6.2 PID调优

如需调整，使用与Intake Drive相同的方法:

1. 从小kP开始 (如10)
2. 逐步增加直到出现振荡
3. 退回到稳定值
4. 微调kV改善响应

---

## 七、WPILib工具链使用

### 7.1 Shuffleboard布局

```
[Feeder/WashingMachineSetpoint] [Feeder/WashingMachineVelocity]
[Feeder/IndexerSetpoint]         [Feeder/IndexerVelocity]
[Feeder/WashingMachineCurrent]  [Feeder/IndexerCurrent]
```

### 7.2 DataLog分析

分析时关注:
- 速度响应时间
- 电流峰值
- 两辊速度同步性

---

## 八、安全注意事项

### 8.1 机械安全

1. **电流限制**: 必须保持30A限制
2. **速度限制**: 避免过快导致球/件飞出
3. **定期检查**: 轴承润滑

### 8.2 操作安全

1. **赛前测试**: 验证两辊转向正确
2. **赛中监控**: 观察电流异常
3. **紧急停止**: 确保可快速停止

---

## 九、代码改进建议

### 9.1 建议改进

1. **速度曲线**: 增加斜坡启动
2. **堵转检测**: 检测到电流过高时自动停止
3. **速度自适应**: 根据Shooter状态调整速度

### 9.2 调优优先级

| 优先级 | 项目 | 影响 |
|--------|------|------|
| P0 | 速度匹配 | 供料可靠性 |
| P1 | 两辊同步 | 不会堆积 |
| P2 | 堵转保护 | 机械安全 |

---

*本文档基于11319_2026_Alphabot_code代码库分析生成*
*最后更新: 2026-03-20*
