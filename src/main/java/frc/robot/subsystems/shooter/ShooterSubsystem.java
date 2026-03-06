// Copyright (c) 2025-2026 Team11319 Polaris
// https://github.com/bbwcQWE
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/** 发射器子系统 - 控制发射机构 包含Hood、Turret和FlyWheel三个子模块 */
public class ShooterSubsystem extends SubsystemBase {

  @AutoLog
  public static class ShooterStateInputs {
    public boolean isReady = false; // 是否准备就绪
    public boolean isAiming = false; // 是否正在瞄准
    public boolean isShooting = false; // 是否正在射击
    public double targetDistance = 0; // 目标距离
    public double hoodAngle = 0; // Hood角度
    public double turretAngle = 0; // 炮塔角度
    public double flywheelSpeed = 0; // 飞轮速度
  }

  private final ShooterStateInputsAutoLogged shooterStateInputs =
      new ShooterStateInputsAutoLogged();

  private final HoodSubsystem hood = new HoodSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final FlyWheelSubsystem flywheel = new FlyWheelSubsystem();
  private final SOTMCalculator sotmCalculator = SOTMCalculator.getInstance();

  private Supplier<AngularVelocity> flywheelVelocitySupplier = () -> DegreesPerSecond.of(0);

  private boolean isReady = false;
  private boolean isAiming = false;
  private boolean isShooting = false;
  private boolean isSOTMEnabled = false;
  private double targetDistance = 0;

  // SOTM相关状态
  private Pose2d currentRobotPose = Pose2d.kZero;
  private ChassisSpeeds currentRobotSpeeds = new ChassisSpeeds();
  private SOTMCalculator.ShotParams sotmParams = null;

  private static final double HOOD_MIN_ANGLE = 20.0; // Hood最小角度
  private static final double HOOD_MAX_ANGLE = 70.0; // Hood最大角度
  private static final double FLYWHEEL_MIN_RPM = 1000.0; // 飞轮最小转速
  private static final double FLYWHEEL_MAX_RPM = 6000.0; // 飞轮最大转速

  // Drive子系统引用（用于获取机器人的位姿和速度）
  private Drive drive = null;

  private void updateInputs() {
    shooterStateInputs.isReady = isReady;
    shooterStateInputs.isAiming = isAiming;
    shooterStateInputs.isShooting = isShooting;
    shooterStateInputs.targetDistance = targetDistance;
    shooterStateInputs.hoodAngle = hood.getAngle().in(Degrees);
    shooterStateInputs.turretAngle = turret.getAngle().in(Degrees);
    shooterStateInputs.flywheelSpeed = flywheel.getVelocity().in(RPM);
  }

  public ShooterSubsystem() {}

  /**
   * 瞄准目标
   *
   * @param distance 目标距离（米）
   * @param targetHeight 目标高度（米）
   * @return 瞄准命令
   */
  public Command aimAtTarget(double distance, double targetHeight) {
    isAiming = true;
    this.targetDistance = distance; // 保存目标距离供 isReadyToShoot() 使用
    Logger.recordOutput("Shooter/TargetDistance", distance);
    Logger.recordOutput("Shooter/TargetHeight", targetHeight);

    double[] shotParams = calculateShotParameters(distance, targetHeight);
    double hoodAngleDeg = shotParams[0];
    double flywheelRPM = shotParams[1];

    Angle hoodAngle = Degrees.of(hoodAngleDeg);
    AngularVelocity flywheelSpeed = RPM.of(flywheelRPM);

    flywheelVelocitySupplier = () -> flywheelSpeed;

    return hood.setAngle(hoodAngle)
        .alongWith(turret.setAngle(Degrees.of(0)))
        .beforeStarting(() -> isAiming = true, hood, turret)
        .andThen(
            () -> {
              isAiming = false;
              isReady = true;
            });
  }

  /**
   * 根据距离计算射击参数（Hood角度和飞轮转速）
   *
   * @param distance 目标距离
   * @param targetHeight 目标高度
   * @return [hoodAngle, flywheelRPM] 数组
   */
  private double[] calculateShotParameters(double distance, double targetHeight) {
    double hoodAngle;
    double flywheelRPM;

    if (distance < 2.0) {
      hoodAngle = 65.0;
      flywheelRPM = 2000.0;
    } else if (distance < 3.0) {
      hoodAngle = 55.0;
      flywheelRPM = 3000.0;
    } else if (distance < 4.0) {
      hoodAngle = 45.0;
      flywheelRPM = 4000.0;
    } else if (distance < 5.0) {
      hoodAngle = 38.0;
      flywheelRPM = 4800.0;
    } else if (distance < 6.0) {
      hoodAngle = 32.0;
      flywheelRPM = 5200.0;
    } else if (distance < 7.0) {
      hoodAngle = 28.0;
      flywheelRPM = 5500.0;
    } else if (distance < 8.0) {
      hoodAngle = 25.0;
      flywheelRPM = 5700.0;
    } else {
      hoodAngle = 22.0;
      flywheelRPM = 5800.0;
    }

    hoodAngle = Math.max(HOOD_MIN_ANGLE, Math.min(HOOD_MAX_ANGLE, hoodAngle));
    flywheelRPM = Math.max(FLYWHEEL_MIN_RPM, Math.min(FLYWHEEL_MAX_RPM, flywheelRPM));

    return new double[] {hoodAngle, flywheelRPM};
  }

  /**
   * 准备射击 - 加速飞轮到目标速度
   *
   * @return 准备射击命令
   */
  public Command prepareToShoot() {
    return flywheel
        .setVelocity(flywheelVelocitySupplier.get())
        .beforeStarting(
            () -> {
              isReady = false;
              isShooting = true;
            },
            flywheel);
  }

  /**
   * 检查是否准备就绪可以射击
   *
   * @return 是否准备就绪
   */
  public boolean isReadyToShoot() {
    double currentRPM = flywheel.getVelocity().in(RPM);
    double targetRPM = flywheelVelocitySupplier.get().in(RPM);
    double rpmError = Math.abs(currentRPM - targetRPM);

    double hoodError =
        Math.abs(hood.getAngle().in(Degrees) - calculateShotParameters(targetDistance, 0)[0]);

    return rpmError < 200 && hoodError < 5 && !isShooting;
  }

  /**
   * 执行射击
   *
   * @return 射击命令
   */
  public Command executeShoot() {
    return prepareToShoot()
        .andThen(
            () -> {
              isShooting = false;
              isReady = false;
            });
  }

  /**
   * 瞄准指定角度
   *
   * @param hoodAngle Hood目标角度
   * @param turretAngle 炮塔目标角度
   * @return 瞄准命令
   */
  public Command aimAt(Angle hoodAngle, Angle turretAngle) {
    return hood.setAngle(hoodAngle).alongWith(turret.setAngle(turretAngle));
  }

  /**
   * 运行发射器（使用预设速度）
   *
   * @return 运行命令
   */
  public Command runShooter() {
    if (flywheelVelocitySupplier == null) {
      DriverStation.reportWarning("Shooter velocity set to null, not running shooter", true);
      return flywheel.setDutyCycle(0);
    }
    return flywheel.setVelocity(flywheelVelocitySupplier.get());
  }

  /**
   * 停止发射器
   *
   * @return 停止命令
   */
  public Command stopShooter() {
    return flywheel.setVelocity(DegreesPerSecond.of(0));
  }

  /**
   * 运行发射器（指定速度）
   *
   * @param velocity 指定速度
   * @return 运行命令
   */
  public Command runShooter(AngularVelocity velocity) {
    if (velocity == null) {
      DriverStation.reportWarning("Shooter velocity set to null, defaulting to 0", true);
      velocity = DegreesPerSecond.of(0);
    }
    return flywheel.setVelocity(velocity);
  }

  /**
   * 设置飞轮速度供应器
   *
   * @param velocitySupplier 速度供应器
   */
  public void setVelocitySupplier(Supplier<AngularVelocity> velocitySupplier) {
    flywheelVelocitySupplier = velocitySupplier;
  }

  /**
   * 获取Hood子系统
   *
   * @return HoodSubsystem实例
   */
  public HoodSubsystem getHood() {
    return hood;
  }

  /**
   * 获取炮塔子系统
   *
   * @return TurretSubsystem实例
   */
  public TurretSubsystem getTurret() {
    return turret;
  }

  /**
   * 获取飞轮子系统
   *
   * @return FlyWheelSubsystem实例
   */
  public FlyWheelSubsystem getFlywheel() {
    return flywheel;
  }

  /**
   * 设置准备状态
   *
   * @param ready 是否准备就绪
   */
  public void setReady(boolean ready) {
    isReady = ready;
  }

  /**
   * 设置瞄准状态
   *
   * @param aiming 是否正在瞄准
   */
  public void setAiming(boolean aiming) {
    isAiming = aiming;
  }

  /**
   * 设置射击状态
   *
   * @param shooting 是否正在射击
   */
  public void setShooting(boolean shooting) {
    isShooting = shooting;
  }

  /**
   * 设置Drive子系统引用（用于SOTM计算）
   *
   * @param drive Drive子系统实例
   */
  public void setDrive(Drive drive) {
    this.drive = drive;
  }

  /**
   * 启用/禁用SOTM模式
   *
   * @param enabled 是否启用
   */
  public void setSOTMEnabled(boolean enabled) {
    this.isSOTMEnabled = enabled;
    if (!enabled) {
      sotmParams = null;
    }
  }

  /**
   * 获取SOTM是否启用
   *
   * @return SOTM是否启用
   */
  public boolean isSOTMEnabled() {
    return isSOTMEnabled;
  }

  /**
   * 获取当前SOTM计算结果
   *
   * @return SOTM射击参数，如果未启用则返回null
   */
  public SOTMCalculator.ShotParams getSOTMParams() {
    return sotmParams;
  }

  /**
   * 执行SOTM瞄准（移动中射击） 注意：建议使用periodic()中的自动计算，只在需要时调用此方法获取结果
   *
   * @return SOTM瞄准命令
   */
  public Command aimSOTM() {
    return run(
        () -> {
          if (drive != null && isSOTMEnabled) {
            // 获取机器人当前位置和速度
            currentRobotPose = drive.getPose();
            currentRobotSpeeds = drive.getFieldSpeeds();

            // 计算SOTM参数
            sotmParams = sotmCalculator.calculate(currentRobotPose, currentRobotSpeeds);

            if (sotmParams != null && sotmParams.isValid()) {
              // 更新速度供应器
              flywheelVelocitySupplier = () -> sotmParams.flywheelRpm();

              isReady = true;
              isAiming = true;
            }
          }
        });
  }

  @Override
  public void periodic() {
    // 更新输入数据
    updateInputs();

    // 如果启用了SOTM模式，更新计算
    // 注意：aimSOTM()命令中也可能触发此计算，这是预期行为
    if (isSOTMEnabled && drive != null) {
      currentRobotPose = drive.getPose();
      currentRobotSpeeds = drive.getFieldSpeeds();
      sotmParams = sotmCalculator.calculate(currentRobotPose, currentRobotSpeeds);

      // 应用SOTM结果到执行器
      if (sotmParams != null && sotmParams.isValid()) {
        hood.setAngleDirect(sotmParams.hoodAngle());
        turret.setAngleDirect(Degrees.of(sotmParams.turretAngle().getDegrees()));
        flywheel.setVelocityDirect(sotmParams.flywheelRpm());
      }
    }

    // 记录日志
    Logger.processInputs("Shooter/State", shooterStateInputs);
  }
}
