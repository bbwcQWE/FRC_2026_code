// Copyright (c) 2025-2026 Team11319 Polaris
// https://github.com/bbwcQWE
// 发射器子系统 - Hood（瞄准机构）

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class HoodSubsystem extends SubsystemBase {

  @AutoLog
  public static class HoodInputs {
    public Angle position = Degrees.of(0);
    public double voltage = 0;
    public double current = 0;
  }

  private final HoodInputsAutoLogged hoodInputs = new HoodInputsAutoLogged();

  private final SparkMax hoodMotor = new SparkMax(16, MotorType.kBrushless);

  private final SmartMotorControllerConfig hoodMotorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
          .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController hoodSMC =
      new SparkWrapper(hoodMotor, DCMotor.getNeo550(1), hoodMotorConfig);

  private final ArmConfig hoodConfig =
      new ArmConfig(hoodSMC)
          .withStartingPosition(Degrees.of(45))
          .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
          .withSoftLimits(Degrees.of(5), Degrees.of(100))
          .withHardLimit(Degrees.of(0), Degrees.of(120))
          .withLength(Inches.of(10))
          .withMass(Pounds.of(2));

  private final Arm hood = new Arm(hoodConfig);

  private void updateInputs() {
    hoodInputs.position = hood.getAngle();
    hoodInputs.voltage = hoodSMC.getVoltage().in(Volts);
    hoodInputs.current = hoodSMC.getStatorCurrent().in(Amps);
  }

  public HoodSubsystem() {}

  public Command setAngle(Angle angle) {
    Logger.recordOutput("Hood/Setpoint", angle);
    return hood.setAngle(angle);
  }

  public void setAngleDirect(Angle angle) {
    Logger.recordOutput("Hood/Setpoint", angle);
    hoodSMC.setPosition(angle);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return hood.setAngle(
        () -> {
          Angle angle = angleSupplier.get();
          Logger.recordOutput("Hood/Setpoint", angle);
          return angle;
        });
  }

  public Angle getAngle() {
    return hoodInputs.position;
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return hood.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return hood.set(dutyCycle);
  }

  public Command sysId() {
    return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Hood", hoodInputs);
    hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }
}
