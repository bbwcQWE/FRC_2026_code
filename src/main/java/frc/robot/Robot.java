// Copyright (c) 2025-2026 11319 Polaris
// https://github.com/bbwcQWE
//
// 基于 Littleton Robotics AdvantageKit TalonFX(S) Swerve 模板
// http://github.com/Mechanical-Advantage
//
// 本项目源代码受BSD许可证约束，详情请参阅LICENSE文件

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** 虚拟机配置为自动运行此类，并按照TimedRobot文档中的描述调用各模式对应的函数。 如果在创建此项目后更改了此类名或包名，还必须更新build.gradle文件。 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {
    // 记录元数据
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "所有更改已提交";
          case 1 -> "存在未提交的更改";
          default -> "未知";
        });

    // 设置数据接收器和回放源
    switch (Constants.currentMode) {
      case REAL:
        // 在真实机器人上运行，记录到USB存储 ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // 运行物理模拟器，记录到NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // 回放日志，设置回放源
        setUseTiming(false); // 全速运行
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // 启动AdvantageKit日志记录器
    Logger.start();

    // 启动USB摄像头自动捕获 (CameraServer)
    // 将摄像头视频流发送到Dashboard以供查看
    CameraServer.startAutomaticCapture();

    // 实例化RobotContainer将执行所有按钮绑定，并在Dashboard上显示自动选择器
    robotContainer = new RobotContainer();
  }

  /** 此函数在所有模式下周期性调用 */
  @Override
  public void robotPeriodic() {
    // 可选：将线程切换到高优先级以改善循环时间
    // (详见模板项目文档)
    // Threads.setCurrentThreadPriority(true, 99);

    // 运行调度器。负责轮询按钮、添加新调度的命令、
    // 运行已调度的命令、移除已完成或中断的命令，
    // 并运行子系统的periodic()方法。
    // 必须从机器人的periodic块调用此方法，Command-based框架才能正常工作。
    CommandScheduler.getInstance().run();

    // 更新Field2d Dashboard - 将机器人Pose显示到Elastic Dashboard
    robotContainer.getField2dDashboard().update();

    // 恢复非实时线程优先级（请勿修改第一个参数）
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** 机器人禁用时调用一次 */
  @Override
  public void disabledInit() {}

  /** 机器人禁用时周期性调用 */
  @Override
  public void disabledPeriodic() {}

  /** 运行由RobotContainer类选择的自动命令 */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // 调度自动命令
    // 注意：预赛模块方向已在getBLineCommand()中处理
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** 自动模式期间周期性调用 */
  @Override
  public void autonomousPeriodic() {}

  /** 启用遥控模式时调用一次 */
  @Override
  public void teleopInit() {
    // 确保当遥控开始时自动模式停止运行。
    // 如果希望自动模式继续运行直到被另一个命令中断，
    // 请删除此行或注释掉。
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** 操作员控制期间周期性调用 */
  @Override
  public void teleopPeriodic() {}

  /** 启用测试模式时调用一次 */
  @Override
  public void testInit() {
    // 测试模式开始时取消所有运行中的命令
    CommandScheduler.getInstance().cancelAll();
  }

  /** 测试模式期间周期性调用 */
  @Override
  public void testPeriodic() {}

  /** 机器人首次启动时调用一次 */
  @Override
  public void simulationInit() {}

  /** 模拟期间周期性调用 */
  @Override
  public void simulationPeriodic() {}
}
