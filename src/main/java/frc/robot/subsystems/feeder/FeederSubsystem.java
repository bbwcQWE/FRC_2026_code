// Copyright (c) 2025-2026 Team11319 Polaris
// https://github.com/bbwcQWE
// 供料子系统 - 控制供料机构和分度器

package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FeederSubsystem extends SubsystemBase {

  @AutoLog
  public static class FeederInputs {
    public AngularVelocity washingMachineVelocity = RPM.of(0);
    public AngularVelocity indexerVelocity = RPM.of(0);
    public double washingMachineVoltage = 0;
    public double indexerVoltage = 0;
    public double washingMachineCurrent = 0;
    public double indexerCurrent = 0;
  }

  private final FeederInputsAutoLogged feederInputs = new FeederInputsAutoLogged();

  private final SparkMax washingMachineMotor = new SparkMax(21, MotorType.kBrushless);

  private final SmartMotorControllerConfig washingMachineConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(50, 0, 0, RPM.of(10000), RotationsPerSecondPerSecond.of(6000))
          .withSimClosedLoopController(
              50, 0, 0, RPM.of(10000), RotationsPerSecondPerSecond.of(6000))
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withTelemetry("WashingMachineMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(20, 1)))
          .withIdleMode(MotorMode.COAST)
          .withMotorInverted(false)
          .withStatorCurrentLimit(Amps.of(30));

  private final SmartMotorController washingMachineSMC =
      new SparkWrapper(washingMachineMotor, DCMotor.getNEO(1), washingMachineConfig);

  private final FlyWheelConfig washingMachineFlywheelConfig =
      new FlyWheelConfig(washingMachineSMC)
          .withDiameter(edu.wpi.first.units.Units.Meters.of(0.0762))
          .withMass(edu.wpi.first.units.Units.Kilograms.of(0.454));

  private final FlyWheel washingMachine = new FlyWheel(washingMachineFlywheelConfig);

  private final TalonFX indexerMotor = new TalonFX(20);

  private final SmartMotorControllerConfig indexerConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(50, 0, 0, RPM.of(10000), RotationsPerSecondPerSecond.of(6000))
          .withSimClosedLoopController(
              50, 0, 0, RPM.of(10000), RotationsPerSecondPerSecond.of(6000))
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(2, 1)))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          .withStatorCurrentLimit(Amps.of(30));

  private final SmartMotorController indexerSMC =
      new TalonFXWrapper(indexerMotor, DCMotor.getKrakenX60(1), indexerConfig);

  private final FlyWheelConfig indexerFlywheelConfig =
      new FlyWheelConfig(indexerSMC)
          .withDiameter(edu.wpi.first.units.Units.Meters.of(0.0508))
          .withMass(edu.wpi.first.units.Units.Kilograms.of(0.227));

  private final FlyWheel indexer = new FlyWheel(indexerFlywheelConfig);

  private void updateInputs() {
    feederInputs.washingMachineVelocity = washingMachine.getSpeed();
    feederInputs.indexerVelocity = indexer.getSpeed();
    feederInputs.washingMachineVoltage = washingMachineSMC.getVoltage().in(Volts);
    feederInputs.indexerVoltage = indexerSMC.getVoltage().in(Volts);
    feederInputs.washingMachineCurrent = washingMachineSMC.getStatorCurrent().in(Amps);
    feederInputs.indexerCurrent = indexerSMC.getStatorCurrent().in(Amps);
  }

  public FeederSubsystem() {}

  public AngularVelocity getWashingMachineVelocity() {
    return feederInputs.washingMachineVelocity;
  }

  public AngularVelocity getIndexerVelocity() {
    return feederInputs.indexerVelocity;
  }

  public Command setWashingMachineVelocity(AngularVelocity speed) {
    Logger.recordOutput("Feeder/WashingMachineSetpoint", speed);
    return washingMachine.setSpeed(speed);
  }

  public Command setIndexerVelocity(AngularVelocity speed) {
    Logger.recordOutput("Feeder/IndexerSetpoint", speed);
    return indexer.setSpeed(speed);
  }

  public void setWashingMachineVelocityDirect(AngularVelocity speed) {
    Logger.recordOutput("Feeder/WashingMachineSetpoint", speed);
    washingMachineSMC.setVelocity(speed);
  }

  public void setIndexerVelocityDirect(AngularVelocity speed) {
    Logger.recordOutput("Feeder/IndexerSetpoint", speed);
    indexerSMC.setVelocity(speed);
  }

  public Command setWashingMachineVelocity(Supplier<AngularVelocity> speed) {
    return washingMachine.setSpeed(
        () -> {
          AngularVelocity v = speed.get();
          Logger.recordOutput("Feeder/WashingMachineSetpoint", v);
          return v;
        });
  }

  public Command setIndexerVelocity(Supplier<AngularVelocity> speed) {
    return indexer.setSpeed(
        () -> {
          AngularVelocity v = speed.get();
          Logger.recordOutput("Feeder/IndexerSetpoint", v);
          return v;
        });
  }

  public Command setWashingMachineDutyCycle(double dutyCycle) {
    Logger.recordOutput("Feeder/WashingMachineDutyCycle", dutyCycle);
    return washingMachine.set(dutyCycle);
  }

  public Command setIndexerDutyCycle(double dutyCycle) {
    Logger.recordOutput("Feeder/IndexerDutyCycle", dutyCycle);
    return indexer.set(dutyCycle);
  }

  public Command runFeeder(AngularVelocity washingMachineSpeed, AngularVelocity indexerSpeed) {
    return setWashingMachineVelocity(washingMachineSpeed)
        .alongWith(setIndexerVelocity(indexerSpeed));
  }

  public Command runFeeder() {
    return runFeeder(RPM.of(500), RPM.of(500));
  }

  public Command stopFeeder() {
    return washingMachine.setSpeed(RPM.of(0)).alongWith(indexer.setSpeed(RPM.of(0)));
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Feeder", feederInputs);
    washingMachine.updateTelemetry();
    indexer.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    washingMachine.simIterate();
    indexer.simIterate();
  }
}
