// Copyright (c) 2025-2026 FRC Team
// https://github.com/bbwcQWE/FRC_2026_code
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.*;
import frc.robot.subsystems.bline.BLinePathFollower;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

/**
 * BLine路径跟随命令工厂类
 *
 * <p>提供便捷的方法来创建和执行BLine路径跟随命令， 支持从JSON文件加载路径和编程方式创建路径。
 */
public final class BLineCommands {

  private BLineCommands() {
    // 私有构造函数，防止实例化
  }

  // ==================== 从JSON文件加载路径的命令 ====================

  /**
   * 加载并跟随JSON路径文件
   *
   * @param bline BLinePathFollower子系统
   * @param pathFilename 路径文件名（不含.json扩展名）
   * @return 路径跟随命令
   */
  public static Command followPathFromFile(BLinePathFollower bline, String pathFilename) {
    // 使用runOnce包装整个命令构建，避免在序列构造时立即执行
    return Commands.runOnce(
            () -> {
              Path path = bline.loadPath(pathFilename);
              Command followCmd = bline.buildFollowCommandWithPoseReset(path);
              followCmd.schedule();
            })
        .andThen(Commands.none());
  }

  /**
   * 加载并跟随JSON路径文件（无姿态重置）
   *
   * @param bline BLinePathFollower子系统
   * @param pathFilename 路径文件名（不含.json扩展名）
   * @return 路径跟随命令
   */
  public static Command followPathFromFileNoReset(BLinePathFollower bline, String pathFilename) {
    return Commands.sequence(
        Commands.runOnce(() -> bline.loadPath(pathFilename)),
        bline.buildFollowCommand(bline.getCurrentPath()));
  }

  /**
   * 加载并跟随JSON路径文件（带自定义翻转逻辑）
   *
   * @param bline BLinePathFollower子系统
   * @param pathFilename 路径文件名（不含.json扩展名）
   * @param shouldFlip 自定义翻转逻辑
   * @return 路径跟随命令
   */
  public static Command followPathFromFileWithFlip(
      BLinePathFollower bline, String pathFilename, Supplier<Boolean> shouldFlip) {
    return Commands.sequence(
        Commands.runOnce(() -> bline.loadPath(pathFilename)),
        bline.buildFollowCommandWithCustomFlip(bline.getCurrentPath(), shouldFlip));
  }

  // ==================== 编程方式创建路径的命令 ====================

  /**
   * 创建并跟随双Waypoint路径
   *
   * @param bline BLinePathFollower子系统
   * @param drive Drive子系统（用于姿态重置）
   * @param startTranslation 起始位置
   * @param startRotation 起始旋转
   * @param endTranslation 结束位置
   * @param endRotation 结束旋转
   * @return 路径跟随命令
   */
  public static Command followTwoPointPath(
      BLinePathFollower bline,
      Drive drive,
      Translation2d startTranslation,
      Rotation2d startRotation,
      Translation2d endTranslation,
      Rotation2d endRotation) {

    Path.Waypoint start = BLinePathFollower.createWaypoint(startTranslation, startRotation);
    Path.Waypoint end = BLinePathFollower.createWaypoint(endTranslation, endRotation);

    return Commands.sequence(
        Commands.runOnce(() -> bline.createPath(start, end)),
        Commands.runOnce(() -> drive.setPose(bline.getPathStartPose(bline.getCurrentPath()))),
        bline.buildFollowCommand());
  }

  /**
   * 创建并跟随多Waypoint路径
   *
   * @param bline BLinePathFollower子系统
   * @param drive Drive子系统（用于姿态重置）
   * @param waypoints 路径Waypoint数组
   * @return 路径跟随命令
   */
  public static Command followMultiPointPath(
      BLinePathFollower bline, Drive drive, Path.Waypoint... waypoints) {

    return Commands.sequence(
        Commands.runOnce(() -> bline.createPath(waypoints)),
        Commands.runOnce(() -> drive.setPose(bline.getPathStartPose(bline.getCurrentPath()))),
        bline.buildFollowCommand());
  }

  /**
   * 创建并跟随带TranslationTarget的路径
   *
   * @param bline BLinePathFollower子系统
   * @param drive Drive子系统（用于姿态重置）
   * @param start 起始Waypoint
   * @param translations 翻译目标点数组
   * @param end 结束Waypoint
   * @return 路径跟随命令
   */
  public static Command followPathWithTranslations(
      BLinePathFollower bline,
      Drive drive,
      Path.Waypoint start,
      Translation2d[] translations,
      Path.Waypoint end) {

    // 计算需要的Waypoint数量
    int totalElements = 1 + translations.length + 1; // start + translations + end
    Path.Waypoint[] allElements = new Path.Waypoint[totalElements];
    allElements[0] = start;
    allElements[totalElements - 1] = end;

    // 复制translation到waypoint数组（使用默认旋转）
    for (int i = 0; i < translations.length; i++) {
      allElements[i + 1] = new Path.Waypoint(translations[i], Rotation2d.kZero);
    }

    return Commands.sequence(
        Commands.runOnce(() -> bline.createPath(allElements)),
        Commands.runOnce(() -> drive.setPose(bline.getPathStartPose(bline.getCurrentPath()))),
        bline.buildFollowCommand());
  }

  // ==================== 带约束的路径命令 ====================

  /**
   * 创建并跟随带速度约束的路径
   *
   * @param bline BLinePathFollower子系统
   * @param drive Drive子系统
   * @param maxVelocity 最大速度 (m/s)
   * @param maxAcceleration 最大加速度 (m/s²)
   * @param waypoints 路径Waypoint数组
   * @return 路径跟随命令
   */
  public static Command followConstrainedPath(
      BLinePathFollower bline,
      Drive drive,
      double maxVelocity,
      double maxAcceleration,
      Path.Waypoint... waypoints) {

    Path.PathConstraints constraints =
        bline
            .createPathConstraints()
            .setMaxVelocityMetersPerSec(maxVelocity)
            .setMaxAccelerationMetersPerSec2(maxAcceleration);

    return Commands.sequence(
        Commands.runOnce(() -> bline.createPath(constraints, waypoints)),
        Commands.runOnce(() -> drive.setPose(bline.getPathStartPose(bline.getCurrentPath()))),
        bline.buildFollowCommand());
  }

  /**
   * 创建并跟随带范围约束的路径
   *
   * @param bline BLinePathFollower子系统
   * @param drive Drive子系统
   * @param constraints 路径约束
   * @param waypoints 路径Waypoint数组
   * @return 路径跟随命令
   */
  public static Command followRangedConstrainedPath(
      BLinePathFollower bline,
      Drive drive,
      Path.PathConstraints constraints,
      Path.Waypoint... waypoints) {

    return Commands.sequence(
        Commands.runOnce(() -> bline.createPath(constraints, waypoints)),
        Commands.runOnce(() -> drive.setPose(bline.getPathStartPose(bline.getCurrentPath()))),
        bline.buildFollowCommand());
  }

  // ==================== 路径序列命令 ====================

  /**
   * 执行路径序列（在路径之间停止）
   *
   * @param bline BLinePathFollower子系统
   * @param drive Drive子系统
   * @param stopCommand 停止命令（在路径之间执行）
   * @param paths 路径数组
   * @return 路径序列命令
   */
  public static Command pathSequence(
      BLinePathFollower bline, Drive drive, Command stopCommand, Path... paths) {

    Command[] followCommands = new Command[paths.length];
    for (int i = 0; i < paths.length; i++) {
      final int index = i;
      followCommands[i] =
          Commands.sequence(
              Commands.runOnce(() -> bline.setCurrentPath(paths[index])),
              bline.buildFollowCommandWithPoseReset(paths[index]),
              stopCommand);
    }

    return Commands.sequence(followCommands);
  }

  /**
   * 连续执行路径序列（无中间停止）
   *
   * @param bline BLinePathFollower子系统
   * @param drive Drive子系统
   * @param paths 路径数组
   * @return 路径序列命令
   */
  public static Command continuousPathSequence(
      BLinePathFollower bline, Drive drive, Path... paths) {

    Command[] followCommands = new Command[paths.length];
    for (int i = 0; i < paths.length; i++) {
      final int index = i;
      followCommands[i] =
          Commands.deadline(
              bline.buildFollowCommandWithPoseReset(paths[index]),
              Commands.waitSeconds(0.05) // 保持命令运行
              );
    }

    return Commands.sequence(followCommands);
  }

  // ==================== 预设路径 ====================

  /**
   * 创建移动到指定位置的预设路径
   *
   * @param bline BLinePathFollower子系统
   * @param drive Drive子系统
   * @param targetTranslation 目标位置
   * @param targetRotation 目标旋转
   * @return 路径跟随命令
   */
  public static Command moveToPosition(
      BLinePathFollower bline,
      Drive drive,
      Translation2d targetTranslation,
      Rotation2d targetRotation) {

    Pose2d currentPose = drive.getPose();

    return followTwoPointPath(
        bline,
        drive,
        currentPose.getTranslation(),
        currentPose.getRotation(),
        targetTranslation,
        targetRotation);
  }

  /**
   * 创建移动到原点的预设路径
   *
   * @param bline BLinePathFollower子系统
   * @param drive Drive子系统
   * @return 路径跟随命令
   */
  public static Command moveToOrigin(BLinePathFollower bline, Drive drive) {
    return moveToPosition(bline, drive, new Translation2d(), Rotation2d.kZero);
  }

  // ==================== 工具方法 ====================

  /**
   * 设置初始模块方向（用于比赛开始前）
   *
   * @param bline BLinePathFollower子系统
   * @param drive Drive子系统
   * @param pathFilename 路径文件名
   * @return 设置模块方向的命令
   */
  public static Command preorientModules(
      BLinePathFollower bline, Drive drive, String pathFilename) {
    return Commands.runOnce(
        () -> {
          bline.loadPath(pathFilename);
          drive.setModuleOrientations(bline.getPathInitialModuleDirection(bline.getCurrentPath()));
        });
  }

  /**
   * 创建路径（工厂方法）
   *
   * @param bline BLinePathFollower子系统
   * @param waypoints 路径Waypoint数组
   * @return Path对象
   */
  public static Path createPath(BLinePathFollower bline, Path.Waypoint... waypoints) {
    return bline.createPath(waypoints);
  }

  /**
   * 创建带约束的路径（工厂方法）
   *
   * @param bline BLinePathFollower子系统
   * @param constraints 路径约束
   * @param waypoints 路径Waypoint数组
   * @return Path对象
   */
  public static Path createConstrainedPath(
      BLinePathFollower bline, Path.PathConstraints constraints, Path.Waypoint... waypoints) {
    return bline.createPath(constraints, waypoints);
  }
}
