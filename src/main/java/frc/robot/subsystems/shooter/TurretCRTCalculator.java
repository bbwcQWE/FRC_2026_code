// Copyright (c) 2025-2026 Team11319 Polaris
// https://github.com/bbwcQWE
// Turret CRT 计算器 - 使用两个CANcoder计算炮塔绝对角度

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

/**
 * Turret CRT 计算器
 *
 * <p>使用两个CANCoder的读数，通过CRT(中国剩余定理)算法计算炮塔的绝对角度。 这样可以消除单一编码器的歧义，实现多圈旋转后的绝对位置跟踪。
 *
 * <p>配置说明: - CANCoder 1: ID 22, 15T齿轮 - CANCoder 2: ID 23, 19T齿轮 - 共享驱动齿轮: 74T - 电机到炮塔减速比: 48.4363
 * - 机械范围: ±190°
 */
public class TurretCRTCalculator {

  // ========== 配置参数 ==========
  // CANCoder IDs
  public static final int CANCODER_1_ID = 22; // 15T 齿轮
  public static final int CANCODER_2_ID = 23; // 19T 齿轮

  // 机械参数 (三级减速)
  // 10→40 (4:1), 22→54 (~2.45:1), 15→74 (~4.93)
  // 总减速比 ≈ 48.4363
  public static final double COMMON_RATIO = 48.4363; // 电机到炮塔的减速比
  public static final int DRIVE_GEAR_TEETH = 74; // 共享驱动齿轮齿数
  public static final int ENCODER_1_PINION = 15; // CANCoder 1 小齿轮齿数
  public static final int ENCODER_2_PINION = 19; // CANCoder 2 小齿轮齿数

  // 机械范围: ±190° = ±0.528 Rotations (注意: 需要允许负值)
  public static final double MIN_MECHANISM_ANGLE = -190.0 / 360.0; // -0.528
  public static final double MAX_MECHANISM_ANGLE = 190.0 / 360.0; // 0.528

  // Offset: 如果在Tuner X中将CANCoder归零到机械0点，填写0.0
  // 如果需要补偿相位差/方向，可能需要填写非零值(正或负)
  public static final double ENCODER_1_OFFSET = 0.0;
  public static final double ENCODER_2_OFFSET = 0.0;

  // 容差: 根据背隙设置 (约 0.5-1 度映射到 encoder2)
  public static final double MATCH_TOLERANCE = 0.002; // ~0.72 度 at encoder2

  // CANCoder 实例
  private final CANcoder canCoder1;
  private final CANcoder canCoder2;

  // EasyCRT 求解器
  private final EasyCRT easyCRT;

  // 状态跟踪
  private boolean isInitialized = false;

  public TurretCRTCalculator() {
    // 初始化 CANCoder
    canCoder1 = new CANcoder(CANCODER_1_ID);
    canCoder2 = new CANcoder(CANCODER_2_ID);

    // 创建 Supplier<Angle> 供 EasyCRT 使用
    Supplier<Angle> encoder1Supplier =
        () -> {
          double value = canCoder1.getAbsolutePosition().getValueAsDouble();
          return Rotations.of(value);
        };

    Supplier<Angle> encoder2Supplier =
        () -> {
          double value = canCoder2.getAbsolutePosition().getValueAsDouble();
          return Rotations.of(value);
        };

    // 配置 EasyCRT
    var easyCRTConfig =
        new EasyCRTConfig(encoder1Supplier, encoder2Supplier)
            .withCommonDriveGear(
                /* commonRatio (mech:drive) */ COMMON_RATIO,
                /* driveGearTeeth */ DRIVE_GEAR_TEETH,
                /* encoder1Pinion */ ENCODER_1_PINION,
                /* encoder2Pinion */ ENCODER_2_PINION)
            .withAbsoluteEncoderOffsets(
                Rotations.of(ENCODER_1_OFFSET), Rotations.of(ENCODER_2_OFFSET))
            .withMechanismRange(
                Rotations.of(MIN_MECHANISM_ANGLE), Rotations.of(MAX_MECHANISM_ANGLE))
            .withMatchTolerance(Rotations.of(MATCH_TOLERANCE))
            .withAbsoluteEncoderInversions(false, false);

    // 创建 CRT 求解器
    easyCRT = new EasyCRT(easyCRTConfig);

    isInitialized = true;
    Logger.recordOutput("TurretCRT/Initialized", true);
  }

  /**
   * 获取计算出的炮塔绝对角度
   *
   * @return Optional<Angle> 计算的角度，如果无效则返回空
   */
  public java.util.Optional<Angle> getAngle() {
    return easyCRT.getAngleOptional();
  }

  /**
   * 获取CRT状态
   *
   * @return 状态字符串
   */
  public String getStatus() {
    if (!isInitialized) {
      return "NOT_INIT";
    }
    return easyCRT.getLastStatus().name();
  }

  /**
   * 检查是否有有效的角度解决方案
   *
   * @return 是否有效
   */
  public boolean isValid() {
    if (!isInitialized) {
      return false;
    }
    // 检查状态是否为OK
    String statusName = easyCRT.getLastStatus().name();
    return "OK".equals(statusName);
  }

  /** 记录诊断信息到Logger */
  public void logDiagnostics() {
    if (!isInitialized) {
      return;
    }

    var status = easyCRT.getLastStatus();
    Logger.recordOutput("TurretCRT/Status", status.name());

    // 记录原始编码器读数
    double enc1Raw = canCoder1.getAbsolutePosition().getValueAsDouble();
    double enc2Raw = canCoder2.getAbsolutePosition().getValueAsDouble();
    Logger.recordOutput("TurretCRT/Encoder1Raw", enc1Raw);
    Logger.recordOutput("TurretCRT/Encoder2Raw", enc2Raw);

    // 记录计算出的角度
    easyCRT
        .getAngleOptional()
        .ifPresent(
            angle -> {
              Logger.recordOutput("TurretCRT/CalculatedAngle", angle.in(Rotations));
            });

    // 记录警告状态
    String statusName = status.name();
    if ("AMBIGUOUS".equals(statusName)) {
      Logger.recordOutput("TurretCRT/Warning", "Multiple valid solutions found");
    } else if ("NO_SOLUTION".equals(statusName)) {
      Logger.recordOutput("TurretCRT/Error", "No valid solution within tolerance");
    }
  }

  /**
   * 使用CRT结果初始化电机控制器
   *
   * @param motor 要初始化的电机控制器 (需要支持 setEncoderPosition)
   */
  public void initializeMotor(Object motor) {
    easyCRT
        .getAngleOptional()
        .ifPresent(
            mechAngle -> {
              // 反射调用 setEncoderPosition 方法
              try {
                var method = motor.getClass().getMethod("setEncoderPosition", Angle.class);
                method.invoke(motor, mechAngle);
                Logger.recordOutput("TurretCRT/MotorInitialized", true);
              } catch (Exception e) {
                Logger.recordOutput("TurretCRT/MotorInitError", e.getMessage());
              }
            });
  }

  /** 获取CAN ID */
  public int getCANCoder1Id() {
    return CANCODER_1_ID;
  }

  public int getCANCoder2Id() {
    return CANCODER_2_ID;
  }

  /** 获取CANCoder实例 (用于调试) */
  public CANcoder getCANCoder1() {
    return canCoder1;
  }

  public CANcoder getCANCoder2() {
    return canCoder2;
  }
}
