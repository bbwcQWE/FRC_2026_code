# RobotContainer 重构方案 A（渐进式拆分）设计文档

## 概述

**项目**: FRC 2026 Robot Code - RobotContainer 重构
**版本**: 1.0
**日期**: 2026-03-18
**状态**: 待实施

## 背景

当前 `RobotContainer.java` 包含 733 行代码，职责混杂：
- 子系统创建（模式切换）
- Zone 配置
- 按钮绑定
- 自动选择器配置
- BLine 路径加载

**核心目标**：
- 降低复杂度，便于理解
- 提高可测试性
- 不影响现有功能

## 约束条件

- **赛季阶段**: 赛季进行中，代码干扰正常测试
- **团队规模**: 单人开发
- **风险偏好**: 低风险

## 方案 A 增强版设计

### 核心理念

仅重组现有代码结构，最小化移动，不改变任何业务逻辑。

### 改动策略

1. **提取私有方法** - 将长代码块提取为方法
2. **分组注释块** - 用注释区分功能区域
3. **提取常量** - 将魔数移到类顶部

### 具体改动

#### 1. 提取子系统创建方法

```java
// 当前位置：构造函数 97-168 行
// 重构后：
private Drive createDrive(Constants.Mode mode) {
    switch (mode) {
        case REAL:
            return new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                ...);
        case SIM:
            ...
    }
}
```

#### 2. 提取 Zone 配置方法

```java
// 当前位置：构造函数 183-207 行
// 重构后：
private void configureZones() {
    poseSupplier = drive::getPose;
    fieldSpeedsSupplier = drive::getFieldSpeeds;
    trenchZone = new Zones.PredictiveXBaseZone(...);
    inTrenchZoneTrigger = ...
}
```

#### 3. 拆分按钮绑定

```java
// 当前：单一方法 337-607 行（270 行）
// 重构后：
private void configureDriverBindings() { ... }
private void configureOperatorBindings() { ... }
private void configureManualModeC() { ... }
```

#### 4. 提取常量

```java
// 新增类常量
private static final double JOYSTICK_DEADZONE = 0.2;
private static final double TURRET_STEP_DEGREES = 5.0;
private static final double HOOD_STEP_DEGREES = 1.0;
```

## 目标结构

```
RobotContainer.java (目标: ~500 行，减少 30%)
├── 字段声明
├── 常量定义 (新增)
├── 构造函数
│   ├── createDrive() [提取]
│   ├── createVision() [提取]
│   ├── createSubsystems() [提取]
│   ├── configureZones() [提取]
│   ├── configureChoosers() [提取]
│   └── configureButtonBindings()
│       ├── configureDriverBindings() [拆分]
│       ├── configureOperatorBindings() [拆分]
│       └── configureManualModeC() [拆分]
└── getter 方法
```

## 风险评估

| 风险项 | 等级 | 缓解措施 |
|--------|------|----------|
| 方法调用顺序错误 | 低 | 保持原调用顺序 |
| 意外改变行为 | 低 | 仅改结构，不改逻辑 |
| 编译错误 | 低 | 使用 IDE 重构工具 |

## 实施计划

| 阶段 | 任务 | 预估时间 |
|------|------|----------|
| 1 | 提取子系统创建方法 | 20 min |
| 2 | 提取 Zone 配置方法 | 15 min |
| 3 | 拆分按钮绑定方法 | 30 min |
| 4 | 提取常量 | 10 min |
| 5 | 编译验证 | 5 min |
| 6 | 功能测试 | 20 min |

**总计**: ~1.5 小时

## 回滚策略

如遇问题，通过 Git 回滚：
```bash
git checkout -- src/main/java/frc/robot/RobotContainer.java
```

## 验收标准

- [ ] 编译通过
- [ ] 所有按钮绑定正常工作
- [ ] 自动模式可正常选择
- [ ] Zone 触发正常工作
- [ ] 代码可读性提升 30%+
