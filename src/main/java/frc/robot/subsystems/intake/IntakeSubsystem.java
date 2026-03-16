// Copyright (c) 2025-2026 Team11319 Polaris
// https://github.com/bbwcQWE
//  Intake子系统 - 控制 intake 机构

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeSubsystem extends SubsystemBase {

  @AutoLog
  public static class IntakeInputs {
    public Angle turnPosition = Degrees.of(0);
    public AngularVelocity driveVelocity = RPM.of(0);
    public double turnVoltage = 0;
    public double driveVoltage = 0;
    public double turnCurrent = 0;
    public double driveCurrent = 0;
  }

  private final IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();

  private final SparkMax turnMotor = new SparkMax(25, MotorType.kBrushless);

  private final SmartMotorControllerConfig turnMotorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          .withSimClosedLoopController(
              4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withTelemetry("IntakeTurnMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          .withStatorCurrentLimit(Amps.of(30))
          .withClosedLoopRampRate(Seconds.of(0.1))
          .withOpenLoopRampRate(Seconds.of(0.1));

  private final SmartMotorController turnSMC =
      new SparkWrapper(turnMotor, DCMotor.getNEO(1), turnMotorConfig);

  private final ArmConfig turnConfig =
      new ArmConfig(turnSMC)
          .withStartingPosition(Degrees.of(0))
          .withTelemetry("IntakeTurnMech", TelemetryVerbosity.HIGH)
          .withSoftLimits(Degrees.of(0), Degrees.of(90))
          .withHardLimit(Degrees.of(-10), Degrees.of(100))
          .withLength(Meters.of(0.3))
          .withMass(Kilograms.of(2));

  private final Arm turn = new Arm(turnConfig);

  private final TalonFX driveMotor = new TalonFX(26);

  private final SmartMotorControllerConfig driveConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(50, 0, 0, RPM.of(10000), RotationsPerSecondPerSecond.of(6000))
          .withSimClosedLoopController(
              50, 0, 0, RPM.of(10000), RotationsPerSecondPerSecond.of(6000))
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withTelemetry("IntakeDriveMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          .withStatorCurrentLimit(Amps.of(30));

  private final SmartMotorController driveSMC =
      new TalonFXWrapper(driveMotor, DCMotor.getKrakenX60(1), driveConfig);

  private final FlyWheelConfig driveFlywheelConfig =
      new FlyWheelConfig(driveSMC).withDiameter(Meters.of(0.0762)).withMass(Kilograms.of(0.5));

  private final FlyWheel drive = new FlyWheel(driveFlywheelConfig);

  private void updateInputs() {
    intakeInputs.turnPosition = turn.getAngle();
    intakeInputs.driveVelocity = drive.getSpeed();
    intakeInputs.turnVoltage = turnSMC.getVoltage().in(Volts);
    intakeInputs.driveVoltage = driveSMC.getVoltage().in(Volts);
    intakeInputs.turnCurrent = turnSMC.getStatorCurrent().in(Amps);
    intakeInputs.driveCurrent = driveSMC.getStatorCurrent().in(Amps);
  }

  public IntakeSubsystem() {}

  public Angle getTurnPosition() {
    return intakeInputs.turnPosition;
  }

  public AngularVelocity getDriveVelocity() {
    return intakeInputs.driveVelocity;
  }

  public Command setTurnAngle(Angle angle) {
    Logger.recordOutput("Intake/TurnSetpoint", angle);
    return turn.setAngle(angle);
  }

  public void setTurnAngleDirect(Angle angle) {
    Logger.recordOutput("Intake/TurnSetpoint", angle);
    turnSMC.setPosition(angle);
  }

  public Command setTurnAngle(Supplier<Angle> angleSupplier) {
    return turn.setAngle(
        () -> {
          Angle angle = angleSupplier.get();
          Logger.recordOutput("Intake/TurnSetpoint", angle);
          return angle;
        });
  }

  public Command setDriveVelocity(AngularVelocity speed) {
    Logger.recordOutput("Intake/DriveSetpoint", speed);
    return drive.setSpeed(speed);
  }

  public void setDriveVelocityDirect(AngularVelocity speed) {
    Logger.recordOutput("Intake/DriveSetpoint", speed);
    driveSMC.setVelocity(speed);
  }

  public Command setDriveVelocity(Supplier<AngularVelocity> speed) {
    return drive.setSpeed(
        () -> {
          AngularVelocity v = speed.get();
          Logger.recordOutput("Intake/DriveSetpoint", v);
          return v;
        });
  }

  public Command setDriveDutyCycle(double dutyCycle) {
    Logger.recordOutput("Intake/DriveDutyCycle", dutyCycle);
    return drive.set(dutyCycle);
  }

  public Command deploy() {
    return setTurnAngle(Degrees.of(90));
  }

  public Command retract() {
    return setTurnAngle(Degrees.of(0));
  }

  public Command intake() {
    return deploy().andThen(setDriveVelocity(RPM.of(500)));
  }

  public Command eject() {
    return setDriveVelocity(RPM.of(-500));
  }

  public Command stop() {
    return turn.setAngle(getTurnPosition()).alongWith(drive.setSpeed(RPM.of(0)));
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Intake", intakeInputs);
    turn.updateTelemetry();
    drive.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    turn.simIterate();
    drive.simIterate();
  }
}
