// Copyright (c) 2025-2026 FRC Team
// https://github.com/bbwcQWE/FRC_2026_code
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.*;
import frc.robot.subsystems.bline.BLinePathFollower;
import frc.robot.subsystems.drive.Drive;

/**
 * BLine路径跟随使用示例
 *
 * <p>这个类展示了如何使用BLine-Lib的各种功能： 1. 从JSON文件加载路径 2. 编程方式创建路径 3. 设置路径约束 4. 构建路径跟随命令
 */
public class BLineExample {

  private final Drive drive;
  private final BLinePathFollower bline;

  public BLineExample(Drive drive, BLinePathFollower bline) {
    this.drive = drive;
    this.bline = bline;
  }

  // ==================== 示例1: 从JSON文件加载路径 ====================

  /** 加载并跟随example_a.json路径文件 自动处理联盟翻转和里程计重置 */
  public Command followJsonPathExample() {
    return Commands.sequence(
        Commands.runOnce(() -> bline.loadPath("example_a")),
        Commands.runOnce(() -> drive.setPose(bline.getPathStartPose(bline.getCurrentPath()))),
        bline.buildFollowCommand());
  }

  /** 加载并跟随simple_move.json路径文件 */
  public Command followSimpleMovePathExample() {
    return Commands.sequence(
        Commands.runOnce(() -> bline.loadPath("simple_move")),
        Commands.runOnce(() -> drive.setPose(bline.getPathStartPose(bline.getCurrentPath()))),
        bline.buildFollowCommand());
  }

  /** 加载并跟随multi_point.json路径文件 */
  public Command followMultiPointPathExample() {
    return Commands.sequence(
        Commands.runOnce(() -> bline.loadPath("multi_point")),
        Commands.runOnce(() -> drive.setPose(bline.getPathStartPose(bline.getCurrentPath()))),
        bline.buildFollowCommand());
  }

  // ==================== 示例2: 编程方式创建路径 ====================

  /** 创建简单的两点路径（起点 -> 终点） */
  public Command createSimplePathExample() {
    // 创建起点（位置 + 旋转）
    Path.Waypoint start =
        new Path.Waypoint(
            new Translation2d(0.5, 0.5), // 起始位置
            Rotation2d.kZero // 起始旋转
            );

    // 创建终点
    Path.Waypoint end =
        new Path.Waypoint(
            new Translation2d(3.0, 0.0), // 结束位置
            new Rotation2d(Math.PI) // 结束旋转（向后）
            );

    // 创建路径
    Path path = new Path(start, end);

    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(path.getStartPose())), bline.buildFollowCommand(path));
  }

  /** 创建多Waypoint路径 */
  public Command createMultiPointPathExample() {
    // 创建多个Waypoint
    Path.Waypoint[] waypoints = {
      new Path.Waypoint(new Translation2d(0.5, 0.5), Rotation2d.kZero),
      new Path.Waypoint(new Translation2d(2.0, 1.0), Rotation2d.kZero),
      new Path.Waypoint(new Translation2d(3.0, 2.0), new Rotation2d(Math.PI / 4)),
      new Path.Waypoint(new Translation2d(2.0, 3.0), new Rotation2d(Math.PI / 2)),
      new Path.Waypoint(new Translation2d(0.5, 4.0), new Rotation2d(Math.PI))
    };

    // 创建路径
    Path path = new Path(waypoints);

    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(path.getStartPose())), bline.buildFollowCommand(path));
  }

  /** 创建L形路径 */
  public Command createLShapedPathExample() {
    Path.Waypoint[] waypoints = {
      new Path.Waypoint(new Translation2d(0.5, 0.5), Rotation2d.kZero),
      new Path.Waypoint(new Translation2d(4.0, 0.5), Rotation2d.kZero),
      new Path.Waypoint(new Translation2d(4.0, 4.0), new Rotation2d(Math.PI / 2))
    };

    Path path = new Path(waypoints);

    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(path.getStartPose())), bline.buildFollowCommand(path));
  }

  // ==================== 示例3: 设置路径约束 ====================

  /**
   * 创建带全局约束的路径
   *
   * <p>使用BLinePathFollower中设置的默认全局约束
   */
  public Command createConstrainedPathExample() {
    Path.Waypoint[] waypoints = {
      new Path.Waypoint(new Translation2d(0.5, 0.5), Rotation2d.kZero),
      new Path.Waypoint(new Translation2d(5.0, 0.5), new Rotation2d(Math.PI))
    };

    Path path = new Path(waypoints);

    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(path.getStartPose())), bline.buildFollowCommand(path));
  }

  // ==================== 示例4: 联盟翻转 ====================

  /** 使用自动联盟翻转 BLinePathFollower已配置为自动根据红蓝联盟翻转路径 */
  public Command followPathWithAutoFlipExample() {
    return Commands.sequence(
        Commands.runOnce(() -> bline.loadPath("example_a")),
        bline.buildFollowCommandWithPoseReset(bline.getCurrentPath()));
  }

  /** 使用自定义翻转逻辑 */
  public Command followPathWithCustomFlipExample() {
    Path path = new Path("example_a");
    return bline.buildFollowCommandWithCustomFlip(
        path,
        () -> {
          // 自定义翻转逻辑：仅当位置在场地右侧时翻转
          return drive.getPose().getX() > 8.0;
        });
  }

  // ==================== 示例5: 路径序列 ====================

  /** 执行路径序列（路径之间有停止） */
  public Command pathSequenceExample(Path path1, Path path2) {
    return Commands.sequence(
        // 第一段路径
        bline.buildFollowCommandWithPoseReset(path1),
        // 停止
        Commands.runOnce(drive::stop),
        // 等待1秒
        Commands.waitSeconds(1.0),
        // 第二段路径
        bline.buildFollowCommandWithPoseReset(path2));
  }

  /** 执行三条路径的序列 */
  public Command threePathSequenceExample(Path path1, Path path2, Path path3) {
    return Commands.sequence(
        bline.buildFollowCommandWithPoseReset(path1),
        Commands.runOnce(drive::stop),
        Commands.waitSeconds(0.5),
        bline.buildFollowCommandWithPoseReset(path2),
        Commands.runOnce(drive::stop),
        Commands.waitSeconds(0.5),
        bline.buildFollowCommandWithPoseReset(path3));
  }

  // ==================== 示例6: 预设置模块方向 ====================

  /** 在自动开始前预设置模块方向 防止微小的初始偏差 */
  public Command preorientModulesExample(Path path) {
    Rotation2d initialDirection = path.getInitialModuleDirection();

    return Commands.runOnce(
        () -> {
          drive.setModuleOrientations(initialDirection);
        });
  }

  // ==================== 示例7: 移动到指定位置 ====================

  /** 移动到指定位置（从当前位置到目标位置） */
  public Command moveToPositionExample(Translation2d targetTranslation, Rotation2d targetRotation) {
    Pose2d currentPose = drive.getPose();

    Path.Waypoint start =
        new Path.Waypoint(currentPose.getTranslation(), currentPose.getRotation());
    Path.Waypoint end = new Path.Waypoint(targetTranslation, targetRotation);

    Path path = new Path(start, end);

    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(path.getStartPose())), bline.buildFollowCommand(path));
  }

  /** 移动到原点 */
  public Command moveToOriginExample() {
    return moveToPositionExample(new Translation2d(), Rotation2d.kZero);
  }

  /** 移动到场地中心 */
  public Command moveToCenterExample() {
    return moveToPositionExample(new Translation2d(8.0, 4.0), Rotation2d.kZero);
  }
}
