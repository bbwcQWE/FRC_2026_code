// Copyright (c) 2025-2026 Team11319 Polaris
// https://github.com/bbwcQWE
// 发射器子系统 - FlyWheel（飞轮）

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlyWheelSubsystem extends SubsystemBase {

  @AutoLog
  public static class FlyWheelInputs {
    public AngularVelocity velocity = DegreesPerSecond.of(0);
    public double voltage = 0;
    public double current = 0;
  }

  private final FlyWheelInputsAutoLogged flyWheelInputs = new FlyWheelInputsAutoLogged();

  private final TalonFX flywheelMotorMain = new TalonFX(17);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40))
          .withFollowers(Pair.of(new TalonFX(18), true));

  private final SmartMotorController motor =
      new TalonFXWrapper(flywheelMotorMain, DCMotor.getKrakenX60(1), motorConfig);

  private final FlyWheelConfig flywheelConfig =
      new FlyWheelConfig(motor)
          .withDiameter(Inches.of(4))
          .withMass(Kilograms.of(2))
          .withUpperSoftLimit(RPM.of(5000));

  private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

  private void updateInputs() {
    flyWheelInputs.velocity = flywheel.getSpeed();
    flyWheelInputs.voltage = motor.getVoltage().in(Volts);
    flyWheelInputs.current = motor.getStatorCurrent().in(Amps);
  }

  public FlyWheelSubsystem() {}

  public AngularVelocity getVelocity() {
    return flyWheelInputs.velocity;
  }

  public Command setVelocity(AngularVelocity speed) {
    Logger.recordOutput("Flywheel/Setpoint", speed);
    return flywheel.setSpeed(speed);
  }

  public void setVelocityDirect(AngularVelocity speed) {
    Logger.recordOutput("Flywheel/Setpoint", speed);
    motor.setVelocity(speed);
  }

  public Command setDutyCycle(double dutyCycle) {
    Logger.recordOutput("Flywheel/DutyCycle", dutyCycle);
    return flywheel.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return flywheel.setSpeed(
        () -> {
          AngularVelocity v = speed.get();
          Logger.recordOutput("Flywheel/Setpoint", v);
          return v;
        });
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return flywheel.set(
        () -> {
          Double d = dutyCycle.get();
          Logger.recordOutput("Flywheel/DutyCycle", d);
          return d;
        });
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Flywheel", flyWheelInputs);
    flywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    flywheel.simIterate();
  }
}
