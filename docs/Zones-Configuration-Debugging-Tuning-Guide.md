# Zones配置系统调试与调优实战指南

## 一、系统架构概览

### 1.1 设计目标

Zones工具类用于定义和检查机器人场地区域，支持:
- 静态区域检测: 检查当前位置是否在区域内
- 预测性区域检测: 考虑速度预测未来位置

### 1.2 核心接口

```
Zone (接口)
├── contains(Supplier<Pose2d>) → Trigger
├── containsTranslation(Supplier<Translation2d>) → Trigger
└── mirroredX/mirroredY() → Zone

PredictiveXZone (继承Zone)
├── willContain(pose, fieldSpeeds, dt) → Trigger
└── willContainTranslation(translation, fieldSpeeds, dt) → Trigger
```

### 1.3 实现类

| 类名 | 功能 |
|------|------|
| BaseZone | 基础矩形区域 |
| PredictiveXBaseZone | 预测性矩形区域 |
| ZoneCollection | 多个区域的OR组合 |
| PredictiveXZoneCollection | 预测性区域集合 |

---

## 二、基础使用指南

### 2.1 创建静态区域

```java
// 创建矩形区域 (xMin, xMax, yMin, yMax) 单位: 米
Zone scoringZone = new BaseZone(0, 2, 3, 5);

// 或使用距离单位
Zone scoringZone = new BaseZone(Meters.of(0), Meters.of(2), Meters.of(3), Meters.of(5));
```

### 2.2 创建预测性区域

```java
// 考虑机器人速度的预测区域
PredictiveXZone approachZone = new PredictiveXBaseZone(0, 3, 2, 6);
```

### 2.3 区域检测

```java
// 静态检测
Trigger inZone = scoringZone.contains(() -> drive.getPose());

// 预测性检测 (0.5秒后位置)
Trigger willBeInZone = approachZone.willContain(
    () -> drive.getPose(),
    () -> drive.getFieldSpeeds(),
    Seconds.of(0.5)
);

// 使用Trigger绑定命令
inZone.onTrue(new ShootCommand());
```

---

## 三、调试指南

### 3.1 区域可视化

**Field2dDashboard集成**:

Zones类提供 `getCorners()` 方法获取区域边界，可用于Field2d可视化:

```java
// 获取区域角点
List<Translation2d> corners = zone.getCorners();
// 返回: [左下, 右下, 右上, 左上]
```

### 3.2 监控信号

在调试时监控以下信号:

| 信号 | 说明 |
|------|------|
| Zone/InZone | 当前是否在区域内 |
| Zone/WillBeInZone | 未来是否会在区域内 |
| Zone/RobotPose | 机器人当前位置 |
| Zone/PredictedPose | 预测位置 |

### 3.3 常见问题排查

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| Trigger不触发 | 坐标系错误 | 确认FRC坐标系 |
| 预测不准确 | dt时间不合适 | 调整预测时间 |
| 区域边界错误 | 坐标顺序错误 | 检查xMin/xMax顺序 |

---

## 四、参数配置

### 4.1 坐标系统一

**FRC坐标系**:
- X轴: 从蓝方联盟站出发，指向对方联盟站
- Y轴: 垂直于X轴，左侧为正(蓝方视角)
- 原点: 蓝方联盟站角落

**场地图**:

```
Y (m)
^
|  16.459m (27ft)
|
|  ← 8.211m (13.5ft) →  (红方)
|      +-----------+
|      |  2.97m    |  (Source)
|      +-----------+
|
|      +-----------+
|      |  2.97m    |  (Amp)
|      +-----------+
|
+------------------+---------> X (m)
0                  16.459m (27ft)
      (蓝方)
```

### 4.2 区域定义示例

#### 4.2.1 得分区域

```java
// 蓝方得分区
Zone blueScoreZone = new BaseZone(1.5, 3.5, 5.5, 7.5);

// 红方得分区 (使用mirroredX)
Zone redScoreZone = blueScoreZone.mirroredX(16.459);
```

#### 4.2.2 禁入区域

```java
// 靠近发言区的禁入区
Zone noDriveZone = new BaseZone(0, 1, 6, 8);
```

#### 4.2.3 预测性避障

```java
// 预测1秒后的位置
PredictiveXZone safeZone = new PredictiveXBaseZone(0, 5, 0, 8);
Trigger safeInOneSecond = safeZone.willContain(
    drive::getPose,
    drive::getFieldSpeeds,
    Seconds.of(1.0)
);
```

### 4.3 预测时间参数

**dt选择原则**:

| 场景 | 推荐dt | 说明 |
|------|--------|------|
| 高速避障 | 0.3s | 快速响应 |
| 一般检测 | 0.5s | 平衡响应 |
| 慢速操作 | 1.0s | 精确检测 |

---

## 五、与子系统集成

### 5.1 与Drive集成

```java
// 定义安全区域
PredictiveXZone safeZone = new PredictiveXBaseZone(0, 3, 2, 5);

// 当预测位置不在安全区时减速
safeZone.willContain(
    drive::getPose,
    drive::getFieldSpeeds,
    Seconds.of(0.5)
).onFalse(drive.setSpeedMultiplier(0.5));
```

### 5.2 与Shooter集成

```java
// 理想射击区域
Zone optimalShootZone = new BaseZone(2, 4, 4, 6);

// 在区域内自动准备射击
optimalShootZone.contains(drive::getPose)
    .onTrue(shooter.prepareToShoot());
```

### 5.3 区域集合

```java
// 组合多个区域 (OR逻辑)
ZoneCollection autoZones = new ZoneCollection()
    .add(new BaseZone(1, 2, 1, 2))
    .add(new BaseZone(3, 4, 3, 4));

// 任意一个区域触发
Trigger inAutoZone = autoZones.contains(drive::getPose);
```

---

## 六、高级应用

### 6.1 预测性导航

```java
// 预测3个时间点
double[] dtValues = {0.25, 0.5, 1.0};
for (double dt : dtValues) {
    Trigger willBeInZone = predictiveZone.willContain(
        poseSupplier,
        speedSupplier,
        Seconds.of(dt)
    );
    // 多重检查
}
```

### 6.2 区域切换

根据比赛阶段切换区域定义:

```java
public class DynamicZones {
    private Zone currentZone;

    public void updateForPhase(GamePhase phase) {
        switch(phase) {
            case AUTO:
                currentZone = autoZone;
                break;
            case TELEOP:
                currentZone = teleopZone;
                break;
        }
    }
}
```

### 6.3 事件记录

```java
// 记录区域进入/退出事件
inZone.onTrue(() -> Logger.recordOutput("Events/EnteredZone", true));
inZone.onFalse(() -> Logger.recordOutput("Events/ExitedZone", true));
```

---

## 七、调试命令与工具

### 7.1 测试Trigger

```java
// 临时添加测试Trigger
new Trigger(() -> true).onTrue(printCommand("Zone triggered!"));
```

### 7.2 Field2d显示

在Shuffleboard中添加Field2d,手动绘制区域边界进行对比。

### 7.3 DataLog分析

使用AdvantageScope:
1. 加载日志文件
2. 同时显示RobotPose和Zone边界
3. 检查触发时机

---

## 八、最佳实践

### 8.1 区域定义原则

1. **留出余量**: 边界比实际需求大0.2-0.5米
2. **对称处理**: 使用mirroredX/Y处理双方
3. **命名清晰**: zone名体现功能

### 8.2 预测性检测建议

1. **短dt优先**: 0.3-0.5秒响应更快
2. **多重验证**: 结合静态+预测
3. **状态缓存**: 避免重复计算

### 8.3 性能考虑

- Trigger在每次调用时重新计算
- 复杂区域组合注意计算开销
- 必要时使用周期性更新

---

## 九、代码示例库

### 9.1 基本区域检测

```java
// 在RobotContainer中
private final Zone shootingZone = new BaseZone(2.5, 4.0, 5.0, 7.0);

// 绑定命令
shootingZone.contains(drive::getPose)
    .whileTrue(shooter.prepareToShoot());
```

### 9.2 预测性避障

```java
private final PredictiveXZone noDriveZone = new PredictiveXBaseZone(0, 1.5, 6, 8);

// 预测检测
noDriveZone.willContain(
    drive::getPose,
    drive::getFieldSpeeds,
    Seconds.of(0.5)
).onTrue(drive.stop());
```

### 9.3 多区域组合

```java
// OR组合
ZoneCollection allScoringZones = new ZoneCollection()
    .add(new BaseZone(1, 2, 1, 2))
    .add(new BaseZone(7, 8, 1, 2))
    .add(new BaseZone(4, 5, 7, 8));

allScoringZones.contains(drive::getPose).whileTrue(runShooterCommand);
```

---

## 十、常见问题FAQ

### Q1: 区域检测坐标与实际不符?
> 检查Field2dDashboard中的坐标显示，对比实际场地测量值。

### Q2: 预测Trigger过于敏感?
> 增大dt时间，或使用静态检测替代。

### Q3: 如何处理双方联盟区域?
> 使用mirroredX()方法基于当前联盟自动翻转。

---

*本文档基于代码库分析生成*
*最后更新: 2026-03-20*
