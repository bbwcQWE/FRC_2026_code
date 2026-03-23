// Copyright (c) 2025-2026 FRC Team
// https://github.com/bbwcQWE/FRC_2026_code
//
// SOTM (Shooting on the Move) 计算器
// 基于Eeshwar/WPILib向量方案实现

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

/**
 * SOTM计算器 - 实现移动中射击的核心计算
 *
 * <p>基于Eeshwar/WPILib向量方案: V_shot = V_target - V_robot
 *
 * <p>核心功能: 1. 延迟补偿 - 将机器人位置向前推移 2. 炮塔偏移补偿 - 考虑炮塔相对于机器人中心的偏移 3. 向量减法 - 计算补偿后的射击向量 4. 逆向查表 -
 * 从所需速度反推距离获取RPM/Hood
 */
public class SOTMCalculator {
  private static SOTMCalculator instance;

  // 射击参数记录
  public record ShotParams(
      Rotation2d turretAngle, // 炮塔目标角度（场相对）
      double turretVelocity, // 炮塔角速度前馈
      Angle hoodAngle, // Hood目标角度
      double hoodVelocity, // Hood角速度前馈
      AngularVelocity flywheelRpm, // 飞轮目标转速
      double distance, // 目标距离
      boolean isValid // 是否有效（距离在范围内）
      ) {}

  // LUT数据结构: distance → {hoodAngle, rpm, timeOfFlight}
  public record ShooterTableEntry(Angle hoodAngle, AngularVelocity rpm, double timeOfFlight) {}

  // 滤波器（用于导数计算）
  private final LinearFilter turretAngleFilter;
  private final LinearFilter hoodAngleFilter;

  // 上一帧的角度（用于计算导数）
  private Rotation2d lastTurretAngle = null;
  private double lastHoodAngle = Double.NaN;

  // 查找表 - 使用TreeMap + 线性插值
  private final TreeMap<Double, ShooterTableEntry> shotTable;
  private final InterpolatingDoubleTreeMap timeOfFlightMap;
  private final InterpolatingDoubleTreeMap velocityToDistanceMap;

  // 延迟补偿参数（秒）
  private double latencyCompensation = 0.10; // 默认100ms

  // 缓存的最新参数
  private ShotParams cachedParams = null;

  // 循环周期（秒）
  private static final double LOOP_PERIOD_SECS = 0.02;

  // LUT有效范围 - 从Constants获取
  private double MIN_DISTANCE = Constants.FieldConstants.SOTM_MIN_DISTANCE;
  private double MAX_DISTANCE = Constants.FieldConstants.SOTM_MAX_DISTANCE;

  private SOTMCalculator() {
    // 初始化滤波器（100ms窗口）
    int filterWindow = (int) (0.1 / LOOP_PERIOD_SECS);
    turretAngleFilter = LinearFilter.movingAverage(filterWindow);
    hoodAngleFilter = LinearFilter.movingAverage(filterWindow);

    // 初始化查找表
    shotTable = new TreeMap<>();
    timeOfFlightMap = new InterpolatingDoubleTreeMap();
    velocityToDistanceMap = new InterpolatingDoubleTreeMap();

    // 填充LUT数据（需根据实际测试调整）
    initializeLUT();
  }

  public static SOTMCalculator getInstance() {
    if (instance == null) {
      instance = new SOTMCalculator();
    }
    return instance;
  }

  /** 线性插值获取LUT值 */
  private ShooterTableEntry getInterpolatedEntry(double distance) {
    if (distance <= MIN_DISTANCE) {
      return shotTable.get(MIN_DISTANCE);
    }
    if (distance >= MAX_DISTANCE) {
      return shotTable.get(MAX_DISTANCE);
    }

    // 找到相邻的两个点
    Double floorKey = shotTable.floorKey(distance);
    Double ceilingKey = shotTable.ceilingKey(distance);

    if (floorKey == null || ceilingKey == null) {
      return shotTable.get(MIN_DISTANCE);
    }

    if (floorKey.equals(ceilingKey)) {
      return shotTable.get(floorKey);
    }

    // 线性插值
    double t = (distance - floorKey) / (ceilingKey - floorKey);
    ShooterTableEntry floor = shotTable.get(floorKey);
    ShooterTableEntry ceiling = shotTable.get(ceilingKey);

    // 插值计算
    double interpolatedHood =
        floor.hoodAngle().in(Degrees)
            + t * (ceiling.hoodAngle().in(Degrees) - floor.hoodAngle().in(Degrees));
    double interpolatedRpm =
        floor.rpm().in(RPM) + t * (ceiling.rpm().in(RPM) - floor.rpm().in(RPM));
    double interpolatedToF =
        floor.timeOfFlight() + t * (ceiling.timeOfFlight() - floor.timeOfFlight());

    return new ShooterTableEntry(
        Degrees.of(interpolatedHood), RPM.of(interpolatedRpm), interpolatedToF);
  }

  /** 初始化查找表数据 注意：ToF数据需通过实际测试获得，此处使用参考值 */
  private void initializeLUT() {
    // 距离 → {hoodAngle, rpm, timeOfFlight}
    // 格式: distance(m), {hoodAngle(deg), rpm, timeOfFlight(s)}
    shotTable.put(1.5, new ShooterTableEntry(Degrees.of(65.0), RPM.of(2000.0), 0.40));
    shotTable.put(2.0, new ShooterTableEntry(Degrees.of(55.0), RPM.of(3000.0), 0.50));
    shotTable.put(2.5, new ShooterTableEntry(Degrees.of(50.0), RPM.of(3500.0), 0.55));
    shotTable.put(3.0, new ShooterTableEntry(Degrees.of(45.0), RPM.of(4000.0), 0.60));
    shotTable.put(3.5, new ShooterTableEntry(Degrees.of(42.0), RPM.of(4400.0), 0.66));
    shotTable.put(4.0, new ShooterTableEntry(Degrees.of(38.0), RPM.of(4800.0), 0.72));
    shotTable.put(4.5, new ShooterTableEntry(Degrees.of(35.0), RPM.of(5100.0), 0.78));
    shotTable.put(5.0, new ShooterTableEntry(Degrees.of(32.0), RPM.of(5200.0), 0.85));
    shotTable.put(5.5, new ShooterTableEntry(Degrees.of(30.0), RPM.of(5400.0), 0.92));
    shotTable.put(6.0, new ShooterTableEntry(Degrees.of(28.0), RPM.of(5500.0), 0.98));

    // 填充ToF表（与shotTable对应）
    timeOfFlightMap.put(1.5, 0.40);
    timeOfFlightMap.put(2.0, 0.50);
    timeOfFlightMap.put(2.5, 0.55);
    timeOfFlightMap.put(3.0, 0.60);
    timeOfFlightMap.put(3.5, 0.66);
    timeOfFlightMap.put(4.0, 0.72);
    timeOfFlightMap.put(4.5, 0.78);
    timeOfFlightMap.put(5.0, 0.85);
    timeOfFlightMap.put(5.5, 0.92);
    timeOfFlightMap.put(6.0, 0.98);

    // 构建逆向映射：velocity → distance
    for (double distance : shotTable.keySet()) {
      ShooterTableEntry entry = shotTable.get(distance);
      double velocity = distance / entry.timeOfFlight();
      velocityToDistanceMap.put(velocity, distance);
    }
  }

  /**
   * 获取当前联盟的Hub位置
   *
   * @return Hub位置（场地坐标系）
   */
  public Translation2d getHubPosition() {
    // 从Constants获取蓝方Hub位置
    Translation2d blueHub = Constants.FieldConstants.BLUE_HUB;

    // 检查当前联盟
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    if (alliance == Alliance.Red) {
      // 红方：翻转X坐标
      double fieldLength =
          Constants.FieldConstants.FIELD_LENGTH.in(edu.wpi.first.units.Units.Meters);
      return new Translation2d(fieldLength - blueHub.getX(), blueHub.getY());
    }

    return blueHub;
  }

  /**
   * 计算SOTM射击参数（主方法）
   *
   * @param robotPose 机器人当前位置
   * @param robotVelocity 机器人当前速度（场相对）
   * @return 射击参数
   */
  public ShotParams calculate(Pose2d robotPose, ChassisSpeeds robotVelocity) {
    return calculate(robotPose, robotVelocity, getHubPosition());
  }

  /**
   * 计算SOTM射击参数（指定目标位置）
   *
   * @param robotPose 机器人当前位置
   * @param robotVelocity 机器人当前速度（场相对）
   * @param targetPos 目标位置（场地坐标）
   * @return 射击参数
   */
  public ShotParams calculate(
      Pose2d robotPose, ChassisSpeeds robotVelocity, Translation2d targetPos) {
    // 获取炮塔偏移
    Translation2d turretOffset = Constants.FieldConstants.TURRET_OFFSET;

    // === Step 1: 延迟补偿 ===
    // 将机器人位置向前推移
    Pose2d futurePose =
        robotPose.exp(
            new Twist2d(
                robotVelocity.vxMetersPerSecond * latencyCompensation,
                robotVelocity.vyMetersPerSecond * latencyCompensation,
                robotVelocity.omegaRadiansPerSecond * latencyCompensation));

    // === Step 2: 炮塔偏移补偿 ===
    // 炮塔位置 = 机器人位置 + 偏移(旋转到场地坐标)
    Translation2d turretPos =
        futurePose.getTranslation().plus(turretOffset.rotateBy(futurePose.getRotation()));

    // === Step 3: 炮塔速度计算（包含离心分量）===
    // V_turret = V_robot + ω × r_offset
    double turretVelX =
        robotVelocity.vxMetersPerSecond - robotVelocity.omegaRadiansPerSecond * turretOffset.getY();
    double turretVelY =
        robotVelocity.vyMetersPerSecond + robotVelocity.omegaRadiansPerSecond * turretOffset.getX();
    Translation2d turretVelocity = new Translation2d(turretVelX, turretVelY);

    // === Step 4: 目标向量 ===
    Translation2d toTarget = targetPos.minus(turretPos);
    double distance = toTarget.getNorm();
    Rotation2d targetAngle = toTarget.getAngle();

    // 检查距离是否在有效范围内
    boolean isValid = distance >= MIN_DISTANCE && distance <= MAX_DISTANCE;

    if (!isValid) {
      // 距离超出范围，返回默认值
      cachedParams =
          new ShotParams(targetAngle, 0.0, Degrees.of(45.0), 0.0, RPM.of(4000.0), distance, false);
      return cachedParams;
    }

    // === Step 5: 查表获取基准参数 ===
    ShooterTableEntry baseline = getInterpolatedEntry(distance);
    double baselineVelocity = distance / baseline.timeOfFlight();

    // === Step 6: 向量减法（核心SOTM公式）===
    // V_shot = V_target - V_turret
    Translation2d targetDir = toTarget.div(distance);
    Translation2d targetVel = targetDir.times(baselineVelocity);
    Translation2d shotVel = targetVel.minus(turretVelocity);

    // === Step 7: 提取结果 ===
    Rotation2d shotAngle = shotVel.getAngle();
    double requiredVelocity = shotVel.getNorm();

    // === Step 8: 逆向查表 ===
    // 从所需速度反推有效距离
    double effectiveDist;
    try {
      effectiveDist = velocityToDistanceMap.get(requiredVelocity);
    } catch (Exception e) {
      // 如果查找失败，使用原始距离
      effectiveDist = distance;
    }
    // 边界处理
    effectiveDist = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, effectiveDist));

    ShooterTableEntry adjustedParams = getInterpolatedEntry(effectiveDist);

    // === Step 9: 计算导数（用于前馈）===
    double turretVelocityFdb = 0.0;
    double hoodVelocityFdb = 0.0;

    if (lastTurretAngle != null && !Double.isNaN(lastHoodAngle)) {
      turretVelocityFdb =
          turretAngleFilter.calculate(
              shotAngle.minus(lastTurretAngle).getRadians() / LOOP_PERIOD_SECS);
      hoodVelocityFdb =
          hoodAngleFilter.calculate(
              (adjustedParams.hoodAngle().in(Degrees) - lastHoodAngle) / LOOP_PERIOD_SECS);
    }

    lastTurretAngle = shotAngle;
    lastHoodAngle = adjustedParams.hoodAngle().in(Degrees);

    // 构建结果
    cachedParams =
        new ShotParams(
            shotAngle,
            turretVelocityFdb,
            adjustedParams.hoodAngle(),
            hoodVelocityFdb,
            adjustedParams.rpm(),
            distance,
            true);

    // 记录日志
    Logger.recordOutput("SOTM/TurretAngle", shotAngle.getDegrees());
    Logger.recordOutput("SOTM/Distance", distance);
    Logger.recordOutput("SOTM/RequiredVelocity", requiredVelocity);
    Logger.recordOutput("SOTM/EffectiveDistance", effectiveDist);
    Logger.recordOutput("SOTM/TurretVelocity", turretVelocityFdb);
    Logger.recordOutput("SOTM/IsValid", isValid);

    return cachedParams;
  }

  /** 获取延迟补偿参数 */
  public double getLatencyCompensation() {
    return latencyCompensation;
  }

  /**
   * 设置延迟补偿参数
   *
   * @param latency 延迟时间（秒）
   */
  public void setLatencyCompensation(double latency) {
    this.latencyCompensation = latency;
  }

  /** 获取缓存的最新参数 */
  public ShotParams getCachedParams() {
    return cachedParams;
  }

  /** 重置计算器状态 */
  public void reset() {
    lastTurretAngle = null;
    lastHoodAngle = Double.NaN;
    cachedParams = null;
    turretAngleFilter.reset();
    hoodAngleFilter.reset();
  }

  /**
   * 更新LUT数据（供外部调用）
   *
   * @param distance 距离（米）
   * @param hoodAngle Hood角度（度）
   * @param rpm 飞轮转速（RPM）
   * @param timeOfFlight 飞行时间（秒）
   */
  public void updateLUT(double distance, double hoodAngle, double rpm, double timeOfFlight) {
    shotTable.put(
        distance, new ShooterTableEntry(Degrees.of(hoodAngle), RPM.of(rpm), timeOfFlight));
    timeOfFlightMap.put(distance, timeOfFlight);

    // 更新逆向映射
    double velocity = distance / timeOfFlight;
    velocityToDistanceMap.put(velocity, distance);
  }

  /**
   * 根据距离获取射击参数（Hood角度和飞轮转速） 此方法暴露查找表数据供ShooterSubsystem使用，消除数据重复
   *
   * @param distance 目标距离（米）
   * @return ShooterTableEntry包含hoodAngle和rpm，如果距离超出范围返回边界值
   */
  public ShooterTableEntry getShotParametersForDistance(double distance) {
    return getInterpolatedEntry(distance);
  }

  /**
   * 获取LUT有效距离范围
   *
   * @return 最小距离
   */
  public double getMinDistance() {
    return MIN_DISTANCE;
  }

  /**
   * 获取LUT有效距离范围
   *
   * @return 最大距离
   */
  public double getMaxDistance() {
    return MAX_DISTANCE;
  }
}
