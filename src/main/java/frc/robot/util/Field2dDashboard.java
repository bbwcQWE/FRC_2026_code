// Copyright (c) 2025-2026 11319 Polaris
// https://github.com/bbwcQWE
//
// 本项目源代码受BSD许可证约束，详情请参阅LICENSE文件

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Field2d Dashboard工具类 用于在Elastic/FRC Dashboard上显示机器人Pose2D和2026 GAME DATA
 *
 * <p>使用方法: 1. 在RobotContainer中创建实例: Field2dDashboard fieldDashboard = new
 * Field2dDashboard(drive::getPose); 2. 在periodic中更新: fieldDashboard.update();
 */
public class Field2dDashboard {
  private final Field2d field2d;
  private final Supplier<Pose2d> poseSupplier;
  private double matchStartTime = -1.0;
  private static final double MATCH_DURATION =
      150.0; // 比赛总时长（秒）: 20s Auto + 110s Teleop + 30s Endgame

  // 2026赛季 - Hub激活时段时间边界（从比赛开始计时）
  private static final double TRANSITION_TIME = 130.0; // 130s后为转换时段，Hub始终激活
  private static final double SHIFT1_END = 130.0; // Shift 1结束时间
  private static final double SHIFT2_END = 105.0; // Shift 2结束时间
  private static final double SHIFT3_END = 80.0; // Shift 3结束时间
  private static final double SHIFT4_END = 55.0; // Shift 4结束时间
  private static final double ENDGAME_START = 30.0; // Endgame开始时间

  /**
   * 构造函数
   *
   * @param poseSupplier 提供当前机器人Pose的Supplier
   */
  public Field2dDashboard(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
    this.field2d = new Field2d();

    // 注册到SmartDashboard/NetworkTables
    SmartDashboard.putData("Field", field2d);
  }

  /**
   * 更新Field2d显示的机器人Pose和Game Data
   *
   * <p>应在机器人periodic方法中调用
   */
  public void update() {
    Pose2d currentPose = poseSupplier.get();
    if (currentPose != null) {
      field2d.setRobotPose(currentPose);
    }

    // 更新MatchTime到SmartDashboard
    SmartDashboard.putNumber("MatchTime", getMatchTime());

    // 更新Game Data到SmartDashboard
    SmartDashboard.putString("GameSpecificMessage", getGameSpecificMessage());

    // 更新Hub激活状态到SmartDashboard
    SmartDashboard.putBoolean("IsHubActive", isHubActive());
  }

  /**
   * 获取当前比赛剩余时间（秒）
   *
   * @return 剩余时间，如果无效返回-1
   */
  public double getMatchTime() {
    // 检测比赛开始并记录开始时间
    if (DriverStation.isEnabled() && matchStartTime < 0) {
      matchStartTime = Timer.getFPGATimestamp();
    }

    // 检测比赛结束（禁用）
    if (DriverStation.isDisabled() && matchStartTime > 0) {
      matchStartTime = -1.0;
    }

    // 如果比赛已启动，计算剩余时间
    if (matchStartTime > 0) {
      double elapsed = Timer.getFPGATimestamp() - matchStartTime;
      double remaining = MATCH_DURATION - elapsed;
      return Math.max(0, remaining);
    }

    return -1.0;
  }

  /**
   * 获取Game Specific Message (2026赛季)
   *
   * @return 游戏数据字符串 ('R' = Red先失活, 'B' = Blue先失活, '' = 尚未接收)
   */
  public String getGameSpecificMessage() {
    try {
      return DriverStation.getGameSpecificMessage();
    } catch (Exception e) {
      return "";
    }
  }

  /**
   * 判断当前联盟的Hub是否处于激活状态 (2026赛季)
   *
   * <p>2026赛季中，根据自动阶段得分情况，某个联盟的Hub会先失活。 Hub激活状态由Game Data和Match Time共同决定。
   *
   * @return true表示当前联盟的Hub激活，false表示失活
   */
  public boolean isHubActive() {
    // 获取当前联盟
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();

    // 如果没有联盟信息，无法确定Hub状态
    if (allianceOpt.isEmpty()) {
      return false;
    }

    Alliance alliance = allianceOpt.get();

    // 自动阶段Hub始终激活
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }

    // 如果不是Teleop阶段，Hub不激活
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // 获取Game Data
    String gameData = getGameSpecificMessage();

    // 如果没有收到Game Data，假设Hub激活（可能是比赛早期）
    if (gameData == null || gameData.isEmpty()) {
      return true;
    }

    // 确定哪个联盟的Hub先失活
    boolean redInactiveFirst;
    switch (gameData.charAt(0)) {
      case 'R':
        redInactiveFirst = true;
        break;
      case 'B':
        redInactiveFirst = false;
        break;
      default:
        // 如果Game Data无效，假设Hub激活
        return true;
    }

    // 计算当前联盟的Hub是否激活
    // Shift 1: Blue激活 if (Red先失活), Red激活 if (Blue先失活)
    boolean shift1BlueActive = !redInactiveFirst;

    // 根据当前联盟和Shift确定Hub激活状态
    boolean blueIsActive;
    boolean ourAllianceIsBlue = (alliance == Alliance.Blue);

    // 获取剩余时间
    double matchTime = getMatchTime();

    // 转换时间边界为从比赛开始计时
    double currentTime = MATCH_DURATION - matchTime;

    // Endgame阶段 (最后30秒)，Hub始终激活
    if (currentTime >= TRANSITION_TIME - ENDGAME_START) {
      return true;
    }

    // Shift 4: 55s-30s
    else if (currentTime >= SHIFT4_END) {
      blueIsActive = !shift1BlueActive;
    }
    // Shift 3: 80s-55s
    else if (currentTime >= SHIFT3_END) {
      blueIsActive = shift1BlueActive;
    }
    // Shift 2: 105s-80s
    else if (currentTime >= SHIFT2_END) {
      blueIsActive = !shift1BlueActive;
    }
    // Shift 1: 130s-105s (但实际从20s自动结束后开始，即150-130=20s后)
    else if (currentTime >= SHIFT1_END) {
      blueIsActive = shift1BlueActive;
    }
    // 自动阶段 (0-20s)
    else {
      return true;
    }

    // 返回当前联盟的Hub状态
    return ourAllianceIsBlue ? blueIsActive : !blueIsActive;
  }

  /**
   * 设置机器人Pose（从外部更新）
   *
   * @param pose 要设置的Pose
   */
  public void setRobotPose(Pose2d pose) {
    field2d.setRobotPose(pose);
  }

  /**
   * 获取Field2d实例
   *
   * @return Field2d实例
   */
  public Field2d getField2d() {
    return field2d;
  }

  /**
   * 获取当前机器人Pose
   *
   * @return 当前Pose
   */
  public Pose2d getRobotPose() {
    return field2d.getRobotPose();
  }
}
