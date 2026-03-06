// Copyright (c) 2025-2026 11319 Polaris
// https://github.com/bbwcQWE
//
// 本项目源代码受BSD许可证约束，详情请参阅LICENSE文件

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.*;
import frc.robot.subsystems.drive.Drive;

/**
 * 动态BLine路径跟随命令
 *
 * <p>当命令开始时，从机器人当前位置作为起点动态生成到目标位置的路径， 并执行路径跟随。此命令在initialize()时获取当前位置，确保目标位置正确。
 */
public class DynamicBLineFollowCommand extends Command {

  private final Drive drive;
  private final FollowPath.Builder pathBuilder;
  private final Translation2d targetTranslation;
  private final Rotation2d targetRotation;

  private FollowPath followCommand;

  /**
   * 创建动态BLine路径跟随命令
   *
   * @param drive Drive子系统
   * @param pathBuilder BLine路径构建器
   * @param targetTranslation 目标位置
   * @param targetRotation 目标朝向（可null表示使用起点朝向）
   */
  public DynamicBLineFollowCommand(
      Drive drive,
      FollowPath.Builder pathBuilder,
      Translation2d targetTranslation,
      Rotation2d targetRotation) {
    this.drive = drive;
    this.pathBuilder = pathBuilder;
    this.targetTranslation = targetTranslation;
    this.targetRotation = targetRotation;

    // 添加drive作为需求
    addRequirements(drive);
  }

  /**
   * 创建动态BLine路径跟随命令（使用默认朝向）
   *
   * @param drive Drive子系统
   * @param pathBuilder BLine路径构建器
   * @param targetTranslation 目标位置
   */
  public DynamicBLineFollowCommand(
      Drive drive, FollowPath.Builder pathBuilder, Translation2d targetTranslation) {
    this(drive, pathBuilder, targetTranslation, null);
  }

  @Override
  public void initialize() {
    // 在命令开始时获取当前位置（此时获取是正确的时间点）
    Pose2d currentPose = drive.getPose();
    Translation2d startTranslation = currentPose.getTranslation();
    Rotation2d startRotation = currentPose.getRotation();

    System.out.println("[DynamicBLine] Starting from: " + startTranslation);
    System.out.println("[DynamicBLine] Going to: " + targetTranslation);

    // 确定目标朝向
    Rotation2d finalRotation;
    if (targetRotation != null) {
      finalRotation = targetRotation;
    } else {
      finalRotation = startRotation;
    }

    // 创建从当前位置到目标位置的路径
    Path dynamicPath =
        new Path(
            new Path.Waypoint(startTranslation, 0.2, startRotation),
            new Path.Waypoint(targetTranslation, 0.2, finalRotation));

    // 构建路径跟随命令
    followCommand = pathBuilder.build(dynamicPath);

    // 初始化跟随命令
    followCommand.initialize();
  }

  @Override
  public void execute() {
    if (followCommand != null) {
      followCommand.execute();
    }
  }

  @Override
  public boolean isFinished() {
    if (followCommand != null) {
      return followCommand.isFinished();
    }
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    if (followCommand != null) {
      followCommand.end(interrupted);
    }
    System.out.println("[DynamicBLine] Command ended, interrupted: " + interrupted);
  }
}
