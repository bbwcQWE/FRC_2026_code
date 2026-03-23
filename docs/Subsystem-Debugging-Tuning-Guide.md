# FRC 2026 Alphabot 子系统调试调优综合指南

## 文档索引

本文档作为各子系统调试调优文档的索引入口，提供快速导航和整体调试策略指导。

---

## 子系统文档列表

| 文档名称 | 适用子系统 | 描述 |
|----------|------------|------|
| [Shooter-Subsystem-Debugging-Tuning-Guide.md](./Shooter-Subsystem-Debugging-Tuning-Guide.md) | Shooter (FlyWheel/Hood/Turret) | 发射器系统完整调试指南 |
| [Intake-Subsystem-Debugging-Tuning-Guide.md](./Intake-Subsystem-Debugging-Tuning-Guide.md) | Intake |  intakes系统调试指南 |
| [Feeder-Subsystem-Debugging-Tuning-Guide.md](./Feeder-Subsystem-Debugging-Tuning-Guide.md) | Feeder | 供料系统调试指南 |
| [Zones-Configuration-Debugging-Tuning-Guide.md](./Zones-Configuration-Debugging-Tuning-Guide.md) | Zones | 区域配置系统使用指南 |

---

## 快速开始

### 调试优先级

建议按以下顺序调试各子系统：

| 优先级 | 子系统 | 调试时间 | 说明 |
|--------|--------|----------|------|
| P0 | Shooter FlyWheel | 30min | 最影响得分 |
| P0 | Shooter Hood | 20min | 落点准确性 |
| P1 | Intake | 15min | 关键取料能力 |
| P1 | Feeder | 15min | 供料流畅度 |
| P2 | Shooter Turret | 20min | 瞄准速度 |
| P2 | Shooter SOTM | 30min | 移动射击 |
| P3 | Zones | 10min | 高级功能 |

### 调试工具清单

- [ ] Shuffleboard / Glass
- [ ] AdvantageScope
- [ ] Phoenix Tuner (CTRE)
- [ ] REV Hardware Client (REV)
- [ ] DataLog Viewer

---

## 通用调试流程

### 1. 硬件检查

1. **CAN连线**: 确认所有电机控制器在线
2. **供电**: 检查电池电压
3. **机械**: 手动转动机构确认无卡滞

### 2. 基础测试

```bash
# 运行代码后，在Shuffleboard中:
# 1. 找到对应子系统的Sendable
# 2. 观察输入信号是否正常
# 3. 发送测试命令
```

### 3. PID调优

使用SysId自动化工具:
```bash
./gradlew simulateJava
# 在Shuffleboard中选择 SysId Routine
```

### 4. 集成测试

各子系统单独正常工作后，进行端到端测试:

```
Intake → Feeder → Shooter → 得分
```

---

## 子系统交互关系

```
        ┌─────────────┐
        │   Vision    │ (目标检测)
        └──────┬──────┘
               │ 目标位置
               ▼
        ┌─────────────┐
        │   Shooter   │ ◄──────────────┐
        │ (Hood/Turret│                │
        │  FlyWheel)  │                │
        └──────┬──────┘                │
               │                       │
        Intake │                Feeder │
        ┌──────▼──────┐         ┌──────▼──────┐
        │   Intake   │────────►│   Feeder    │
        └─────────────┘         └─────────────┘
```

---

## 关键参数速查

### Shooter

| 参数 | 文件位置 | 推荐范围 |
|------|----------|----------|
| FlyWheel kP | FlyWheelSubsystem.java:52 | 20-40 |
| Hood kP | HoodSubsystem.java:53 | 0.0001-0.0003 |
| Turret kP | TurretSubsystem.java:59 | 3-6 |
| Velocity Tolerance | ShooterSubsystem.java:63 | 150-300 RPM |
| Hood Angle Tolerance | ShooterSubsystem.java:64 | 1.0-2.5° |

### Intake

| 参数 | 文件位置 | 推荐范围 |
|------|----------|----------|
| Turn kP | IntakeSubsystem.java:63 | 3-6 |
| Drive kP | IntakeSubsystem.java:96 | 30-80 |
| Intake Speed | IntakeSubsystem.java:186 | 400-600 RPM |
| Eject Speed | IntakeSubsystem.java:190 | -400 RPM |

### Feeder

| 参数 | 文件位置 | 推荐范围 |
|------|----------|----------|
| Washing Machine kP | FeederSubsystem.java:54 | 30-80 |
| Indexer kP | FeederSubsystem.java:80 | 30-80 |
| Default Speed | FeederSubsystem.java:174 | 400-600 RPM |

---

## 常见问题汇总

### Q: 电机不转动
1. 检查CAN连线
2. 检查供电
3. 查看DriverStation错误信息

### Q: 响应不准确
1. 检查编码器
2. 调低P增益
3. 检查机械间隙

### Q: 振荡/抖动
1. 降低P增益
2. 检查机械松动
3. 增加D增益(谨慎)

### Q: 温度过高
1. 降低工作频率
2. 检查电流限制
3. 改善散热

---

## 文档更新日志

| 日期 | 更新内容 |
|------|----------|
| 2026-03-20 | 初始版本，涵盖4个子系统文档 |

---

## 贡献指南

如需更新本文档:
1. 编辑对应子系统文档
2. 更新索引表格
3. 提交PR

---

*本索引基于 11319_2026_Alphabot_code 代码库*
*最后更新: 2026-03-20*
