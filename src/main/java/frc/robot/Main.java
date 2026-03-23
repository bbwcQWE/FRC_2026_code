// Copyright (c) 2025-2026 FRC Team
// https://github.com/bbwcQWE/FRC_2026_code
//
// 基于 Littleton Robotics AdvantageKit TalonFX(S) Swerve 模板
// http://github.com/Mechanical-Advantage
//
// 本项目源代码受BSD许可证约束，详情请参阅LICENSE文件

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** 请勿在此类中添加任何静态变量，或进行任何初始化。除非您知道自己在做什么， 请勿修改此文件，除非需要更改startRobot调用中的参数类。 */
public final class Main {
  private Main() {}

  /**
   * 主初始化函数。请勿在此处执行任何初始化操作。
   *
   * <p>如果更改了主机器人类，请更改参数类型。
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
