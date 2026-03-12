// Copyright (c) 2025-2026 Team11319 Polaris
// https://github.com/bbwcQWE
//
// 发射器命令 - 提供各种发射器相关的命令

package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.FlyWheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** 发射器命令类 */
public class ShooterCommands {

  // 使用 ShooterSubsystem 中的容差参数
  private static final double VELOCITY_TOLERANCE_RPM =
      ShooterSubsystem.VELOCITY_TOLERANCE_RPM; // 速度容差 (RPM)

  private ShooterCommands() {}

  /**
   * 完整射击命令 - 等待飞轮达到目标转速,等待Hood到位,启动Feeder推送弹丸,射击完成后停止
   *
   * @param shooter ShooterSubsystem实例
   * @param feeder FeederSubsystem实例
   * @return 射击命令
   */
  public static Command shoot(ShooterSubsystem shooter, FeederSubsystem feeder) {
    return Commands.sequence(
        // 步骤1: 准备射击 (加速飞轮)
        prepareToShoot(shooter)
            .until(shooter::isReadyToShoot)
            .withTimeout(5.0) // 添加5秒超时保护，防止飞轮无法达到目标速度时卡住
            .withName("Shoot-WaitForReady"),

        // 步骤2: 启动Feeder推送弹丸
        Commands.run(
                () -> {
                  Logger.recordOutput("ShooterCommands/Shooting", true);
                  feeder.runFeeder();
                },
                feeder)
            .withTimeout(1.0)
            .withName("Shoot-Feed"),

        // 步骤3: 停止Feeder
        Commands.runOnce(
                () -> {
                  Logger.recordOutput("ShooterCommands/Shooting", false);
                  feeder.stopFeeder();
                  shooter.setShooting(false);
                  shooter.setReady(false);
                },
                feeder,
                shooter)
            .withName("Shoot-Stop"));
  }

  /**
   * 预热飞轮命令 - 设置目标转速,加速飞轮直到达到目标速度
   *
   * @param shooter ShooterSubsystem实例
   * @return 预热命令
   */
  public static Command prepareToShoot(ShooterSubsystem shooter) {
    return Commands.runOnce(
            () -> {
              Logger.recordOutput("ShooterCommands/PrepareToShoot", true);
              shooter.setShooting(true);
              shooter.setReady(false);
            },
            shooter)
        .andThen(shooter.prepareToShoot())
        .withName("PrepareToShoot");
  }

  /**
   * 瞄准目标命令 - 根据距离计算射击参数,控制Hood和Turret到位
   *
   * @param shooter ShooterSubsystem实例
   * @param distanceSupplier 目标距离供应器 (米)
   * @return 瞄准命令
   */
  public static Command aimAtTarget(ShooterSubsystem shooter, Supplier<Double> distanceSupplier) {
    return Commands.runOnce(
            () -> {
              Logger.recordOutput("ShooterCommands/Aiming", true);
              shooter.setAiming(true);
            },
            shooter)
        .andThen(shooter.aimAtTarget(distanceSupplier.get(), 0.0))
        .andThen(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("ShooterCommands/Aiming", false);
                  shooter.setAiming(false);
                }))
        .withName("AimAtTarget");
  }

  /**
   * 瞄准指定距离的目标 (固定距离值)
   *
   * @param shooter ShooterSubsystem实例
   * @param distance 目标距离 (米)
   * @return 瞄准命令
   */
  public static Command aimAtTarget(ShooterSubsystem shooter, double distance) {
    return aimAtTarget(shooter, () -> distance);
  }

  /**
   * 停止射击系统命令 - 停止飞轮,停止Feeder,重置状态标志
   *
   * @param shooter ShooterSubsystem实例
   * @param feeder FeederSubsystem实例
   * @return 停止命令
   */
  public static Command stopShooter(ShooterSubsystem shooter, FeederSubsystem feeder) {
    return Commands.runOnce(
            () -> {
              Logger.recordOutput("ShooterCommands/StopShooter", true);
              shooter.setReady(false);
              shooter.setAiming(false);
              shooter.setShooting(false);
            },
            shooter)
        .andThen(shooter.stopShooter())
        .andThen(feeder.stopFeeder())
        .withName("StopShooter");
  }

  /**
   * 预热命令 (持续运行直到停止) - 保持飞轮在目标转速
   *
   * @param shooter ShooterSubsystem实例
   * @param velocitySupplier 目标速度供应器
   * @return 预热命令
   */
  public static Command revShooter(
      ShooterSubsystem shooter, Supplier<AngularVelocity> velocitySupplier) {
    return Commands.run(
            () -> {
              Logger.recordOutput("ShooterCommands/Revving", true);
              AngularVelocity velocity = velocitySupplier.get();
              shooter.getFlywheel().setVelocityDirect(velocity);
            },
            shooter.getFlywheel())
        .withName("RevShooter");
  }

  /**
   * 预热命令 - 使用固定转速
   *
   * @param shooter ShooterSubsystem实例
   * @param rpm 目标转速 (RPM)
   * @return 预热命令
   */
  public static Command revShooter(ShooterSubsystem shooter, double rpm) {
    return revShooter(shooter, () -> RPM.of(rpm));
  }

  /**
   * 预热命令 - 使用默认速度
   *
   * <p>注意: 此方法会调用 shooter.runShooter()，会覆盖之前通过 aimAtTarget 设置的速度供应器。 如果需要使用 aimAtTarget 设置的速度，请直接调用
   * shooter.runShooter() 或确保先调用 aimAtTarget。
   *
   * @param shooter ShooterSubsystem实例
   * @return 预热命令
   */
  public static Command revShooter(ShooterSubsystem shooter) {
    return shooter.runShooter();
  }

  /**
   * 手动射击命令 - 使用固定速度和供料速度
   *
   * @param shooter ShooterSubsystem实例
   * @param feeder FeederSubsystem实例
   * @param flywheelRPM 飞轮目标转速 (RPM)
   * @return 手动射击命令
   */
  public static Command manualShoot(
      ShooterSubsystem shooter, FeederSubsystem feeder, double flywheelRPM) {
    return Commands.sequence(
        // 设置飞轮速度
        Commands.runOnce(
            () -> {
              Logger.recordOutput("ShooterCommands/ManualShoot", true);
              shooter.setShooting(true);
              shooter.setReady(false);
            },
            shooter),

        // 加速飞轮
        shooter.getFlywheel().setVelocity(RPM.of(flywheelRPM)),

        // 等待飞轮达到目标速度
        Commands.waitUntil(
            () -> {
              double currentRPM = shooter.getFlywheel().getVelocity().in(RPM);
              return Math.abs(currentRPM - flywheelRPM) < VELOCITY_TOLERANCE_RPM;
            }),

        // 启动供料
        feeder.runFeeder().withTimeout(1.0),

        // 停止
        Commands.runOnce(
            () -> {
              Logger.recordOutput("ShooterCommands/ManualShoot", false);
              feeder.stopFeeder();
              shooter.setShooting(false);
              shooter.setReady(false);
            },
            feeder,
            shooter));
  }

  /**
   * 快速射击命令 - 假设飞轮已经预热,直接供料
   *
   * @param shooter ShooterSubsystem实例
   * @param feeder FeederSubsystem实例
   * @return 快速射击命令
   */
  public static Command quickShoot(ShooterSubsystem shooter, FeederSubsystem feeder) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              Logger.recordOutput("ShooterCommands/QuickShoot", true);
              shooter.setShooting(true);
            },
            shooter),

        // 启动供料
        feeder.runFeeder().withTimeout(0.5),

        // 停止
        Commands.runOnce(
            () -> {
              Logger.recordOutput("ShooterCommands/QuickShoot", false);
              feeder.stopFeeder();
              shooter.setShooting(false);
            },
            feeder,
            shooter));
  }

  /**
   * 手动设置Hood角度
   *
   * @param hood HoodSubsystem实例
   * @param angleSupplier 角度供应器 (度)
   * @return 设置命令
   */
  public static Command setHoodAngle(HoodSubsystem hood, Supplier<Double> angleSupplier) {
    return Commands.run(
        () -> {
          double angleDeg = angleSupplier.get();
          hood.setAngleDirect(edu.wpi.first.units.Units.Degrees.of(angleDeg));
        },
        hood);
  }

  /**
   * 手动设置炮塔角度
   *
   * @param turret TurretSubsystem实例
   * @param angleSupplier 角度供应器 (度)
   * @return 设置命令
   */
  public static Command setTurretAngle(TurretSubsystem turret, Supplier<Double> angleSupplier) {
    return Commands.run(
        () -> {
          double angleDeg = angleSupplier.get();
          turret.setAngleDirect(edu.wpi.first.units.Units.Degrees.of(angleDeg));
        },
        turret);
  }

  /**
   * 手动设置飞轮速度
   *
   * @param flywheel FlyWheelSubsystem实例
   * @param velocitySupplier 速度供应器 (RPM)
   * @return 设置命令
   */
  public static Command setFlywheelVelocity(
      FlyWheelSubsystem flywheel, Supplier<Double> velocitySupplier) {
    return Commands.run(
        () -> {
          double rpm = velocitySupplier.get();
          flywheel.setVelocityDirect(RPM.of(rpm));
        },
        flywheel);
  }

  /**
   * 手动设置供料速度
   *
   * @param feeder FeederSubsystem实例
   * @param washingMachineRPM Washing Machine速度 (RPM)
   * @param indexerRPM Indexer速度 (RPM)
   * @return 设置命令
   */
  public static Command setFeederVelocity(
      FeederSubsystem feeder, double washingMachineRPM, double indexerRPM) {
    return feeder
        .setWashingMachineVelocity(RPM.of(washingMachineRPM))
        .alongWith(feeder.setIndexerVelocity(RPM.of(indexerRPM)));
  }
}
