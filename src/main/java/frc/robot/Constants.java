// Copyright (c) 2025-2026 FRC Team
// https://github.com/bbwcQWE/FRC_2026_code
//
// 基于 Littleton Robotics AdvantageKit TalonFX(S) Swerve 模板
// http://github.com/Mechanical-Advantage
//
// 本项目源代码受BSD许可证约束，详情请参阅LICENSE文件

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * 定义AdvantageKit运行时模式的类。在roboRIO上运行时模式始终为"real"。 修改"simMode"的值可在"sim"（物理模拟）和"replay"（从文件回放日志）之间切换。
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** 在真实机器人上运行 */
    REAL,

    /** 运行物理模拟器 */
    SIM,

    /** 从日志文件回放 */
    REPLAY
  }

  // 控制周期，用于能量计算
  public static final double loopPeriodSecs = 0.02; // 20ms

  public static final class FieldConstants {
    // FRC场地尺寸 (英寸)
    public static final double FIELD_LENGTH_INCHES = 650.12;
    public static final double FIELD_WIDTH_INCHES = 316.64;

    // 转换为米
    public static final Distance FIELD_LENGTH = Inches.of(FIELD_LENGTH_INCHES);
    public static final Distance FIELD_WIDTH = Inches.of(FIELD_WIDTH_INCHES);

    // trench区域 (需要根据你的机器人实际场地调整)
    public static final double TRENCH_X_START = 0.0; // 米
    public static final double TRENCH_X_END = 5.5;
    public static final double TRENCH_Y_MIN = 2.5;
    public static final double TRENCH_Y_MAX = 4.5;

    // 预测时间
    public static final Time DUCK_TIME = Seconds.of(0.2);

    // Hood角度
    public static final Angle MIN_HOOD_ANGLE = Degrees.of(14); // 收回位置
    public static final Angle DEFAULT_HOOD_ANGLE = Degrees.of(45); // 默认位置

    // 炮塔相对于机器人中心的偏移量（米）
    // 正X向右，正Y向上（基于机器人视角）
    public static final double TURRET_OFFSET_X = 0.14806;
    public static final double TURRET_OFFSET_Y = 0.15861;
    public static final Translation2d TURRET_OFFSET =
        new Translation2d(TURRET_OFFSET_X, TURRET_OFFSET_Y);

    // ========== SOTM相关常量 ==========

    // Hub位置（蓝方坐标系）- FRC 2026 Rebuilt
    public static final double BLUE_HUB_X = 4.6;
    public static final double BLUE_HUB_Y = 4.03;
    public static final Translation2d BLUE_HUB = new Translation2d(BLUE_HUB_X, BLUE_HUB_Y);

    // SOTM LUT有效距离范围（米）
    public static final double SOTM_MIN_DISTANCE = 0.1;
    public static final double SOTM_MAX_DISTANCE = 6.0;

    // 默认延迟补偿（秒）- 包含相机、处理、CAN、电机响应
    // TODO: 需根据实际系统测试标定
    public static final double SOTM_DEFAULT_LATENCY = 0.10;
  }
}
