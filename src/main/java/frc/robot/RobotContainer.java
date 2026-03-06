// Copyright (c) 2025-2026 11319 Polaris
// https://github.com/bbwcQWE
//
// 基于 Littleton Robotics AdvantageKit TalonFX(S) Swerve 模板
// http://github.com/Mechanical-Advantage
//
// 本项目源代码受BSD许可证约束，详情请参阅LICENSE文件

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DynamicBLineFollowCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.*;
import frc.robot.subsystems.bline.BLinePathFollower;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Zones;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * 这是声明机器人大部分组件的类。由于Command-based是一种"声明式"范式， 实际上很少有机器人逻辑应该在{@link Robot}的periodic方法中处理（除了调度器调用）。
 * 相反，机器人的结构（包括子系统、OI设备和按钮映射）应该在这里声明。
 */
public class RobotContainer {
  // 子系统
  private final Drive drive;
  private final Vision vision;
  private final BLinePathFollower blinePathFollower;
  private HoodSubsystem hood;
  private final FeederSubsystem feeder;
  private final IntakeSubsystem intake;
  // private final ShooterSubsystem shooter;

  // Zone相关的Suppliers - 在构造函数中初始化
  private Supplier<Pose2d> poseSupplier;
  private Supplier<ChassisSpeeds> fieldSpeedsSupplier;

  // 炮塔位置Suppliers（考虑偏移）- 在构造函数中初始化
  private Supplier<Translation2d> turretPoseSupplier;
  private Supplier<ChassisSpeeds> turretFieldSpeedsSupplier;

  // Zone定义 - 根据实际场地尺寸调整
  public Zones.PredictiveXZone trenchZone;

  // Zone Trigger - 用于命令绑定
  public Trigger inTrenchZoneTrigger;

  // 控制器
  // 主控制器(插槽0) - 驾驶员控制驱动
  private final CommandXboxController mainController = new CommandXboxController(0);

  // 操作控制器(插槽1) - 操作手柄控制(如取消自动、特殊功能)
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard输入
  private final LoggedDashboardChooser<Command> autoChooser;

  /** 机器人的容器。包含子系统、OI设备和命令。 */
  public RobotContainer() {
    // 更新AprilTag场地选择（根据Dashboard选择）
    VisionConstants.updateFieldFromChooser();

    switch (Constants.currentMode) {
      case REAL:
        // 真实机器人，实例化硬件IO实现
        // ModuleIOTalonFX适用于带有TalonFX驱动、TalonFX转向和CANcoder的模块
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // 使用Limelight相机实例化视觉子系统
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

        // 初始化feeder和intake子系统
        feeder = new FeederSubsystem();
        intake = new IntakeSubsystem();

        // 初始化shooter子系统
        // shooter = new ShooterSubsystem();
        break;

      case SIM:
        // 模拟机器人，实例化物理模拟IO实现
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        // 使用模拟相机实例化视觉子系统
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));

        // 初始化feeder和intake子系统
        feeder = new FeederSubsystem();
        intake = new IntakeSubsystem();

        // 初始化shooter子系统
        // shooter = new ShooterSubsystem();
        break;

      default:
        // 回放机器人，禁用IO实现
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // 在回放模式下禁用视觉
        vision = new Vision(drive::addVisionMeasurement);

        // 初始化feeder和intake子系统
        feeder = new FeederSubsystem();
        intake = new IntakeSubsystem();

        // 初始化shooter子系统
        // shooter = new ShooterSubsystem();
        break;
    }

    // 注册视觉子系统以启用periodic()调用
    vision.register();

    // 注册feeder和intake子系统
    feeder.register();
    intake.register();

    // 注册shooter子系统以启用periodic()调用
    // shooter.register();

    // Initialize HoodSubsystem
    hood = new HoodSubsystem();
    hood.register();

    // 初始化Zone相关的Suppliers
    poseSupplier = drive::getPose;
    fieldSpeedsSupplier = drive::getFieldSpeeds;

    // 初始化炮塔位置Suppliers（考虑偏移）
    turretPoseSupplier =
        () -> drive.getPose().getTranslation().plus(Constants.FieldConstants.TURRET_OFFSET);
    turretFieldSpeedsSupplier = drive::getFieldSpeeds;

    // 初始化Trench Zone (使用Constants中的值)
    // x: 0到5.5米, y: 2.5到4.5米
    trenchZone =
        new Zones.PredictiveXBaseZone(
            Constants.FieldConstants.TRENCH_X_START,
            Constants.FieldConstants.TRENCH_X_END,
            Constants.FieldConstants.TRENCH_Y_MIN,
            Constants.FieldConstants.TRENCH_Y_MAX);

    // 创建基于Translation2d的Zone Trigger - 使用炮塔位置
    inTrenchZoneTrigger =
        trenchZone.willContainTranslation(
            turretPoseSupplier, turretFieldSpeedsSupplier, Constants.FieldConstants.DUCK_TIME);

    // 初始化BLine路径跟随子系统
    blinePathFollower = new BLinePathFollower(drive);

    // 使用PathPlanner设置自动例程
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // 添加PathPlanner"None"选项作为默认（将回退到BLine检查）
    autoChooser.addDefaultOption("None (Use BLine)", Commands.runOnce(drive::stop, drive));

    // 配置BLine路径选择器
    configureBLineAutoChooser();

    // 添加系统选择器（PathPlanner / BLine / None）
    configureSystemChooser();

    // 设置SysId例程
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // 配置按钮绑定
    configureButtonBindings();
  }

  /** BLine路径选择器 - 在dashboard上显示 */
  private final SendableChooser<String> blinePathChooser = new SendableChooser<>();

  /** 配置选择器的BLine自动例程 */
  private void configureBLineAutoChooser() {
    // 添加默认选项
    blinePathChooser.setDefaultOption("None", "");

    // 尝试多种方式获取路径文件
    boolean foundPaths = false;

    // 方法1: 尝试从deploy目录读取 (运行时)
    String[] possiblePaths = {
      "/home/lvuser/deploy/autos/paths", // RoboRIO路径
      "src/main/deploy/autos/paths", // 开发环境相对路径
      "deploy/autos/paths" // 另一种可能的路径
    };

    for (String basePath : possiblePaths) {
      java.io.File pathsDir = new java.io.File(basePath);
      if (pathsDir.exists() && pathsDir.isDirectory()) {
        java.io.File[] jsonFiles = pathsDir.listFiles((dir, name) -> name.endsWith(".json"));
        if (jsonFiles != null && jsonFiles.length > 0) {
          for (java.io.File file : jsonFiles) {
            String pathName = file.getName().replace(".json", "");
            blinePathChooser.addOption(pathName, pathName);
            foundPaths = true;
          }
          break;
        }
      }
    }

    // 方法2: 如果文件系统方法失败，尝试从classpath资源读取
    if (!foundPaths) {
      try {
        java.net.URL resourceUrl = getClass().getClassLoader().getResource("autos/paths");
        if (resourceUrl != null) {
          java.io.File pathsDir = new java.io.File(resourceUrl.getPath());
          if (pathsDir.exists() && pathsDir.isDirectory()) {
            java.io.File[] jsonFiles = pathsDir.listFiles((dir, name) -> name.endsWith(".json"));
            if (jsonFiles != null) {
              for (java.io.File file : jsonFiles) {
                String pathName = file.getName().replace(".json", "");
                blinePathChooser.addOption(pathName, pathName);
                foundPaths = true;
              }
            }
          }
        }
      } catch (Exception e) {
        // 忽略错误，继续使用默认选项
      }
    }

    // 如果仍然没有找到路径，手动添加已知路径（基于项目结构）
    if (!foundPaths) {
      // 这些是项目中的已知路径文件
      String[] knownPaths = {
        "auto_Down", "example_a", "example_b", "multi_point", "ranged_constraints", "simple_move"
      };
      for (String pathName : knownPaths) {
        blinePathChooser.addOption(pathName, pathName);
      }
    }

    // 注册到NetworkTable使其显示在dashboard上
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(
        "BLine Path Chooser", blinePathChooser);
  }

  /** 自动系统选择器 - 选择使用哪个系统 */
  private final SendableChooser<String> systemChooser = new SendableChooser<>();

  /** 配置系统选择器 */
  private void configureSystemChooser() {
    // 添加系统选项
    systemChooser.addOption("PathPlanner", "pathplanner");
    systemChooser.addOption("BLine", "bline");
    systemChooser.addOption("None (Stop)", "none");

    // 设置默认
    systemChooser.setDefaultOption("PathPlanner", "pathplanner");

    // 注册到NetworkTable
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(
        "Auto System Chooser", systemChooser);
  }

  /**
   * 使用此方法定义按钮到命令的映射。按钮可以通过实例化{@link GenericHID} 或其子类（{@link edu.wpi.first.wpilibj.Joystick}或{@link
   * XboxController}）创建， 然后传递给{@link edu.wpi.first.wpilibj2.command.button.JoystickButton}。
   */
  private void configureButtonBindings() {
    // 默认命令，常规场相对驱动
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -mainController.getLeftY(),
            () -> -mainController.getLeftX(),
            () -> -mainController.getRightX()));

    // 按住A按钮时锁定到0°
    mainController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -mainController.getLeftY(),
                () -> -mainController.getLeftX(),
                () -> Rotation2d.kZero));

    // 按住Y按钮时锁定到45度方向
    mainController
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -mainController.getLeftY(),
                () -> -mainController.getLeftX(),
                () -> Rotation2d.fromDegrees(45)));

    // 按下X按钮时切换到X形态
    mainController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // 按下B按钮时重置陀螺仪到0°
    mainController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // 按下RB按钮时，从当前位置动态生成BLine路径到目标位置 (6.4, 6.5)
    mainController
        .rightBumper()
        .onTrue(
            new DynamicBLineFollowCommand(
                drive, blinePathFollower.getPathBuilder(), new Translation2d(6.4, 6.5)));

    // ==================== BLine 自动中断功能 ====================
    // 主手柄左摇杆 - 当驾驶员实际移动摇杆时（超过阈值）取消自动并切换到手动驾驶
    // 使用Trigger检测摇杆是否超过阈值（0.2），只有故意的操作才会触发
    new Trigger(
            () -> {
              double leftY = Math.abs(mainController.getLeftY());
              double leftX = Math.abs(mainController.getLeftX());
              // 超过0.2阈值才认为是"打杆"操作
              return leftY > 0.2 || leftX > 0.2;
            })
        .whileTrue(
            Commands.run(
                () -> {
                  // 取消所有正在执行的自动命令（包括BLine路径跟随）
                  CommandScheduler.getInstance().cancelAll();

                  // 手动驾驶由默认命令处理，这里只需要取消自动命令
                  // 当Trigger条件满足时，WPILib会自动使用默认命令
                },
                drive));

    // ==================== BLine 自动中断功能 (操作控制器) ====================
    // 使用操作手柄(插槽1)上的按钮来取消自动命令
    // 这样不会与驾驶员的驱动控制冲突

    // 操作手柄左肩键(LB) - 取消所有自动命令并停止
    operatorController
        .leftBumper()
        .onTrue(
            Commands.run(
                () -> {
                  // 取消所有自动命令
                  CommandScheduler.getInstance().cancelAll();
                  // 停止驱动
                  drive.stop();
                  System.out.println("[BLine] Auto command cancelled by operator");
                },
                drive));

    // 操作手柄右肩键(RB) - 紧急停止，锁定模块
    operatorController
        .rightBumper()
        .onTrue(
            Commands.run(
                () -> {
                  // 取消所有自动命令
                  CommandScheduler.getInstance().cancelAll();
                  // 锁定模块（X形态）
                  drive.stopWithX();
                  System.out.println("[BLine] Emergency stop activated - X formation");
                },
                drive));

    // 操作手柄A按钮 - 取消自动并切换到手动驾驶
    operatorController
        .a()
        .onTrue(
            Commands.run(
                () -> {
                  // 取消所有自动命令
                  CommandScheduler.getInstance().cancelAll();
                  System.out.println("[BLine] Switched to manual control");
                },
                drive));

    // Y按钮：跟随示例路径
    // 注意：BLine路径跟随命令应该在自动模式或需要时调用getBLineCommand()方法
    // 这里暂时禁用，因为路径文件需要在运行时加载

    // SysId测试用于shooter子系统
    // controller.y().whileTrue(shooter.getHood().sysId());

    // Hood自动收回 - 当进入trench区域时收回hood
    if (hood != null) {
      inTrenchZoneTrigger.onTrue(hood.setAngle(Constants.FieldConstants.MIN_HOOD_ANGLE));
      inTrenchZoneTrigger.onFalse(hood.setAngle(Constants.FieldConstants.DEFAULT_HOOD_ANGLE));
    }
  }

  /**
   * 获取BLinePathFollower子系统
   *
   * @return BLinePathFollower实例
   */
  public BLinePathFollower getBLinePathFollower() {
    return blinePathFollower;
  }

  /**
   * 获取Drive子系统
   *
   * @return Drive实例
   */
  public Drive getDrive() {
    return drive;
  }

  // /**
  //  * 获取Feeder子系统
  //  *
  //  * @return FeederSubsystem实例
  //  */
  // public FeederSubsystem getFeeder() {
  //   return feeder;
  // }

  // /**
  //  * 获取Intake子系统
  //  *
  //  * @return IntakeSubsystem实例
  //  */
  // public IntakeSubsystem getIntake() {
  //   return intake;
  // }

  /**
   * 使用此方法将自动命令传递给主{@link Robot}类。
   *
   * @return 自动模式下运行的命令
   */
  public Command getAutonomousCommand() {
    // 获取选择的系统
    String selectedSystem = systemChooser.getSelected();

    // 默认为PathPlanner
    if (selectedSystem == null) {
      selectedSystem = "pathplanner";
    }

    // 根据选择的系统执行
    switch (selectedSystem) {
      case "pathplanner":
        // PathPlanner系统
        Command ppCommand = autoChooser.get();
        if (ppCommand != null) {
          return ppCommand;
        }
        // 如果PathPlanner返回空命令，返回停止
        return Commands.runOnce(drive::stop, drive);

      case "bline":
        // BLine系统
        String blinePath = blinePathChooser.getSelected();
        if (blinePath != null && !blinePath.isEmpty()) {
          return getBLineCommand(blinePath);
        }
        // 如果BLine没有选择，返回停止
        return Commands.runOnce(drive::stop, drive);

      case "none":
      default:
        // None - 停止
        return Commands.runOnce(drive::stop, drive);
    }
  }

  /**
   * 构建BLine路径跟随命令（供自动模式使用）
   *
   * @param pathFilename 路径文件名（不含.json扩展名）
   * @return 路径跟随命令，如果路径为空则返回停止命令
   */
  public Command getBLineCommand(String pathFilename) {
    // 验证路径文件名
    if (pathFilename == null || pathFilename.isEmpty()) {
      return Commands.runOnce(drive::stop, drive);
    }

    try {
      // 加载路径
      Path path = blinePathFollower.loadPath(pathFilename);

      // 验证路径是否成功加载
      if (path == null) {
        System.err.println("BLine: Failed to load path - " + pathFilename);
        return Commands.runOnce(drive::stop, drive);
      }

      // Pre-Match Module Orientation: 获取初始模块方向并在命令开始前设置
      // 这可以防止自动开始时模块旋转导致的微小偏差
      Rotation2d initialDirection = path.getInitialModuleDirection();

      // 构建命令（带姿态重置和模块方向预设置）
      Command followCommand = blinePathFollower.buildFollowCommandWithPoseReset(path);

      // 在命令开始时预设置模块方向
      return Commands.sequence(
          Commands.runOnce(() -> drive.setModuleOrientations(initialDirection)), followCommand);
    } catch (Exception e) {
      System.err.println(
          "BLine: Error building command for " + pathFilename + ": " + e.getMessage());
      return Commands.runOnce(drive::stop, drive);
    }
  }
}
