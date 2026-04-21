// Copyright (c) 2025-2026 FRC Team
// https://github.com/bbwcQWE/FRC_2026_code
//
// 本项目源代码受BSD许可证约束，详情请参阅LICENSE文件

package frc.robot.subsystems.bline;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.BLine.*;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * BLine路径跟随子系统
 *
 * <p>提供基于BLine-Lib的路径跟随功能，支持： - 从JSON文件加载路径 - 编程方式创建路径 - 自动联盟翻转 - 路径跟随命令构建
 */
public class BLinePathFollower extends SubsystemBase {

  private final Drive drive;

  // BLine路径构建器
  private FollowPath.Builder pathBuilder;

  // 路径实例
  private Path currentPath;

  // ==================== PID控制器参数 ====================
  // 平移控制器 - 控制基于剩余距离的速度
  private static final double TRANSLATION_P = 5.0;
  private static final double TRANSLATION_I = 0.0;
  private static final double TRANSLATION_D = 0.0;

  // 旋转控制器 - 控制全向旋转朝向目标
  private static final double ROTATION_P = 3.0;
  private static final double ROTATION_I = 0.0;
  private static final double ROTATION_D = 0.0;

  // 横向误差控制器 - 最小化与路径线的偏差
  private static final double CROSS_TRACK_P = 2.0;
  private static final double CROSS_TRACK_I = 0.0;
  private static final double CROSS_TRACK_D = 0.0;

  // ==================== 全局约束默认值 ====================
  private static final double DEFAULT_MAX_VELOCITY_MPS = 4.5;
  private static final double DEFAULT_MAX_ACCELERATION_MPS2 = 7.0;
  private static final double DEFAULT_MAX_VELOCITY_DEG_PS = 720.0;
  private static final double DEFAULT_MAX_ACCELERATION_DEG_PS2 = 1500.0;
  private static final double DEFAULT_END_TRANSLATION_TOLERANCE_M = 0.03;
  private static final double DEFAULT_END_ROTATION_TOLERANCE_DEG = 2.0;
  private static final double DEFAULT_INTERMEDIATE_HANDOFF_RADIUS_M = 0.2;

  /**
   * 构造函数
   *
   * @param drive Drive subsystem实例
   */
  public BLinePathFollower(Drive drive) {
    this.drive = drive;
    initialize();
  }

  /** 初始化BLine系统 */
  private void initialize() {
    // 设置全局默认约束
    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(
            DEFAULT_MAX_VELOCITY_MPS,
            DEFAULT_MAX_ACCELERATION_MPS2,
            DEFAULT_MAX_VELOCITY_DEG_PS,
            DEFAULT_MAX_ACCELERATION_DEG_PS2,
            DEFAULT_END_TRANSLATION_TOLERANCE_M,
            DEFAULT_END_ROTATION_TOLERANCE_DEG,
            DEFAULT_INTERMEDIATE_HANDOFF_RADIUS_M));

    // 创建路径构建器
    pathBuilder =
        new FollowPath.Builder(
                drive, // Subsystem requirement
                drive::getPose, // Current pose supplier
                drive::getChassisSpeeds, // Current speeds supplier
                drive::runVelocity, // Drive consumer
                new PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D),
                new PIDController(ROTATION_P, ROTATION_I, ROTATION_D),
                new PIDController(CROSS_TRACK_P, CROSS_TRACK_I, CROSS_TRACK_D))
            .withDefaultShouldFlip(); // Auto-flip for red alliance

    // 配置日志记录（可选）
    configureLogging();
  }

  /** 配置BLine日志记录 使用AdvantageKit Logger记录路径跟随状态 */
  private void configureLogging() {
    // Double类型日志（速度、位置误差等）
    FollowPath.setDoubleLoggingConsumer(
        pair -> {
          Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });

    // Boolean类型日志（是否完成、是否翻转等）
    FollowPath.setBooleanLoggingConsumer(
        pair -> {
          Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });

    // Pose类型日志（目标位姿等）
    FollowPath.setPoseLoggingConsumer(
        pair -> {
          Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });

    // Translation数组日志（路径点等）
    FollowPath.setTranslationListLoggingConsumer(
        pair -> {
          Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });
  }

  /**
   * 加载路径从JSON文件
   *
   * @param filename 路径文件名（不含.json扩展名）
   * @return 加载的Path对象
   */
  public Path loadPath(String filename) {
    currentPath = new Path(filename);
    return currentPath;
  }

  /**
   * 编程方式创建路径（使用Waypoint）
   *
   * @param elements 路径Waypoint元素
   * @return 创建的Path对象
   */
  public Path createPath(Path.Waypoint... elements) {
    currentPath = new Path(elements);
    return currentPath;
  }

  /**
   * 创建带约束的路径（使用Waypoint）
   *
   * @param constraints 路径约束
   * @param elements 路径Waypoint元素
   * @return 创建的Path对象
   */
  public Path createPath(Path.PathConstraints constraints, Path.Waypoint... elements) {
    currentPath = new Path(constraints, elements);
    return currentPath;
  }

  /**
   * 构建路径跟随命令
   *
   * @param path 要跟随的路径
   * @return FollowPath命令
   */
  public FollowPath buildFollowCommand(Path path) {
    return pathBuilder.build(path);
  }

  /**
   * 构建路径跟随命令（使用当前路径）
   *
   * @return FollowPath命令
   */
  public FollowPath buildFollowCommand() {
    if (currentPath == null) {
      throw new IllegalStateException("当前没有设置路径，请先加载或创建路径");
    }
    return pathBuilder.build(currentPath);
  }

  /**
   * 创建从当前位置到目标位置的路径并构建跟随命令
   *
   * @param targetTranslation 目标位置 (Translation2d)
   * @param targetRotation 目标朝向 (Rotation2d，可为null表示保持当前朝向)
   * @return 路径跟随命令
   */
  public Command createAndFollowPathTo(Translation2d targetTranslation, Rotation2d targetRotation) {
    // 获取当前位置作为起点
    Pose2d currentPose = drive.getPose();
    Translation2d startTranslation = currentPose.getTranslation();
    Rotation2d startRotation = currentPose.getRotation();

    // 如果没有指定目标朝向，使用起点朝向
    if (targetRotation == null) {
      targetRotation = startRotation;
    }

    // 创建从当前位置到目标位置的路径
    // 使用两个waypoint：起点和终点
    Path dynamicPath =
        new Path(
            new Path.Waypoint(startTranslation, 0.2, startRotation),
            new Path.Waypoint(targetTranslation, 0.2, targetRotation));

    currentPath = dynamicPath;

    // 构建并返回跟随命令（不带姿态重置，保持当前位姿）
    return pathBuilder.build(dynamicPath);
  }

  /**
   * 创建从当前位置到目标位置的路径并构建跟随命令（使用默认朝向）
   *
   * @param targetTranslation 目标位置 (Translation2d)
   * @return 路径跟随命令
   */
  public Command createAndFollowPathTo(Translation2d targetTranslation) {
    return createAndFollowPathTo(targetTranslation, null);
  }

  /**
   * 获取路径构建器（用于自定义命令）
   *
   * @return FollowPath.Builder实例
   */
  public FollowPath.Builder getPathBuilder() {
    return pathBuilder;
  }

  /**
   * 构建带姿态重置的路径跟随命令
   *
   * @param path 要跟随的路径
   * @return FollowPath命令
   */
  public FollowPath buildFollowCommandWithPoseReset(Path path) {
    return pathBuilder.withPoseReset(drive::setPose).build(path);
  }

  /**
   * 构建带自定义翻转逻辑的路径跟随命令
   *
   * @param path 要跟随的路径
   * @param shouldFlip 自定义翻转逻辑
   * @return FollowPath命令
   */
  public FollowPath buildFollowCommandWithCustomFlip(Path path, Supplier<Boolean> shouldFlip) {
    return pathBuilder.withShouldFlip(shouldFlip).build(path);
  }

  /**
   * 获取路径的起始姿态（用于里程计重置）
   *
   * @param path 路径
   * @return 起始Pose2d
   */
  public Pose2d getPathStartPose(Path path) {
    return path.getStartPose();
  }

  /**
   * 获取路径的初始模块方向
   *
   * @param path 路径
   * @return 初始Rotation2d
   */
  public Rotation2d getPathInitialModuleDirection(Path path) {
    return path.getInitialModuleDirection();
  }

  /**
   * 翻转路径（手动）
   *
   * @param path 要翻转的路径
   */
  public void flipPath(Path path) {
    path.flip();
  }

  /**
   * 取消路径翻转
   *
   * @param path 路径
   */
  public void undoFlipPath(Path path) {
    path.undoFlip();
  }

  /**
   * 配置路径约束构建器
   *
   * @return PathConstraints构建器
   */
  public Path.PathConstraints createPathConstraints() {
    return new Path.PathConstraints();
  }

  /**
   * 创建全局约束
   *
   * @param maxVelocityMps 最大速度 (m/s)
   * @param maxAccelerationMps2 最大加速度 (m/s²)
   * @param maxVelocityDegPs 最大旋转速度 (deg/s)
   * @param maxAccelerationDegPs2 最大旋转加速度 (deg/s²)
   * @param endTranslationToleranceM 终点平移容差 (m)
   * @param endRotationToleranceDeg 终点旋转容差 (deg)
   * @param intermediateHandoffRadiusM 中间交接半径 (m)
   * @return DefaultGlobalConstraints对象
   */
  public Path.DefaultGlobalConstraints createDefaultGlobalConstraints(
      double maxVelocityMps,
      double maxAccelerationMps2,
      double maxVelocityDegPs,
      double maxAccelerationDegPs2,
      double endTranslationToleranceM,
      double endRotationToleranceDeg,
      double intermediateHandoffRadiusM) {
    return new Path.DefaultGlobalConstraints(
        maxVelocityMps,
        maxAccelerationMps2,
        maxVelocityDegPs,
        maxAccelerationDegPs2,
        endTranslationToleranceM,
        endRotationToleranceDeg,
        intermediateHandoffRadiusM);
  }

  /**
   * 更新全局约束
   *
   * @param constraints 新的全局约束
   */
  public void updateDefaultGlobalConstraints(Path.DefaultGlobalConstraints constraints) {
    Path.setDefaultGlobalConstraints(constraints);
  }

  /**
   * 获取当前路径
   *
   * @return 当前Path对象，可能为null
   */
  public Path getCurrentPath() {
    return currentPath;
  }

  /**
   * 设置当前路径
   *
   * @param path 路径
   */
  public void setCurrentPath(Path path) {
    this.currentPath = path;
  }

  /**
   * 创建范围约束
   *
   * @param value 约束值
   * @param startOrdinal 起始序号（包含）
   * @param endOrdinal 结束序号（包含）
   * @return RangedConstraint对象
   */
  public Path.RangedConstraint createRangedConstraint(
      double value, int startOrdinal, int endOrdinal) {
    return new Path.RangedConstraint(value, startOrdinal, endOrdinal);
  }

  // ==================== 静态工厂方法 - 创建路径元素 ====================

  /**
   * 创建Waypoint（位置+旋转）
   *
   * @param translation 位置
   * @param rotation 旋转
   * @return Waypoint对象
   */
  public static Path.Waypoint createWaypoint(Translation2d translation, Rotation2d rotation) {
    return new Path.Waypoint(translation, rotation);
  }

  /**
   * 创建Waypoint（使用Pose2d）
   *
   * @param pose 位姿
   * @return Waypoint对象
   */
  public static Path.Waypoint createWaypoint(Pose2d pose) {
    return new Path.Waypoint(pose);
  }

  /**
   * 创建TranslationTarget（仅位置）
   *
   * @param translation 位置
   * @return TranslationTarget对象
   */
  public static Path.TranslationTarget createTranslationTarget(Translation2d translation) {
    return new Path.TranslationTarget(translation);
  }

  /**
   * 创建TranslationTarget（使用坐标）
   *
   * @param x X坐标
   * @param y Y坐标
   * @return TranslationTarget对象
   */
  public static Path.TranslationTarget createTranslationTarget(double x, double y) {
    return new Path.TranslationTarget(x, y);
  }

  /**
   * 创建RotationTarget（仅旋转）
   *
   * @param rotation 旋转
   * @param tRatio 沿路径段的位置比例 (0-1)
   * @return RotationTarget对象
   */
  public static Path.RotationTarget createRotationTarget(Rotation2d rotation, double tRatio) {
    return new Path.RotationTarget(rotation, tRatio);
  }

  /**
   * 获取平移PID控制器参数
   *
   * @return [P, I, D]数组
   */
  public static double[] getTranslationPID() {
    return new double[] {TRANSLATION_P, TRANSLATION_I, TRANSLATION_D};
  }

  /**
   * 获取旋转PID控制器参数
   *
   * @return [P, I, D]数组
   */
  public static double[] getRotationPID() {
    return new double[] {ROTATION_P, ROTATION_I, ROTATION_D};
  }

  /**
   * 获取横向误差PID控制器参数
   *
   * @return [P, I, D]数组
   */
  public static double[] getCrossTrackPID() {
    return new double[] {CROSS_TRACK_P, CROSS_TRACK_I, CROSS_TRACK_D};
  }
}
