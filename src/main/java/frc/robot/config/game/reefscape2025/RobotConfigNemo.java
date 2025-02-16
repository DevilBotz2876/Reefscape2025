package frc.robot.config.game.reefscape2025;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.io.implementations.motor.MotorIOBase.MotorIOBaseSettings;
import frc.robot.io.implementations.motor.MotorIOSparkMax;
import frc.robot.io.implementations.motor.MotorIOSparkMax.SparkMaxSettings;
import frc.robot.io.implementations.motor.MotorIOTalonFx;
import frc.robot.io.implementations.motor.MotorIOTalonFx.TalonFxSettings;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.implementations.motor.ArmMotorSubsystem;
import frc.robot.subsystems.implementations.motor.FlywheelMotorSubsystem;
import frc.robot.subsystems.interfaces.ArmV2.ArmSettings;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Flywheel.FlywheelSettings;

/* Override Nemo specific constants here */
public class RobotConfigNemo extends RobotConfig {
  public RobotConfigNemo() {
    super(false, true, true, true, false, true);

    // Nemo has a Swerve drive train
    Drive.Constants.rotatePidKp = 0.025;
    Drive.Constants.rotatePidKi = 0.0;
    Drive.Constants.rotatePidKd = 0.0;
    DriveBase.Constants.rotatePidErrorInDegrees = 1;
    drive = new DriveSwerveYAGSL("yagsl/nemo");

    // Coral Arm
    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      // 20:1 gear box, 30 teeth on the arm cog and 15 teeth on the motor cog
      motorSettings.motor.gearing = 20 * (30.0 / 15.0);
      motorSettings.motor.inverted = true; // false for Sim
      motorSettings.pid = new PIDController(0.35, 0.0, 0);

      ArmSettings armSettings = new ArmSettings();
      armSettings.minAngleInDegrees = -90;
      armSettings.maxAngleInDegrees = 60;
      armSettings.startingAngleInDegrees = armSettings.minAngleInDegrees;
      armSettings.feedforward = new ArmFeedforward(0.0, 0.16, 0.0, 0.0);
      armSettings.color = new Color8Bit(Color.kRed);
      armSettings.armLengthInMeters = 0.5;
      armSettings.armMassInKg = 1.0;
      armSettings.motor = DCMotor.getKrakenX60(1);
      armSettings.simulateGravity = true;
      armSettings.maxVelocityInDegreesPerSecond = 360 * 2;
      armSettings.maxAccelerationInDegreesPerSecondSquared = 360;

      TalonFxSettings talonFxSettings = new TalonFxSettings();
      talonFxSettings.canId = 21;

      coralArm =
          new ArmMotorSubsystem(
              // new MotorIOArmStub(motorSettings, armSettings), "Coral", armSettings);
              new MotorIOTalonFx(motorSettings, talonFxSettings), "Coral", armSettings);
    }

    // Algae Flywheel
    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      motorSettings.motor.gearing = 1; // TODO get the actual gearing
      motorSettings.motor.inverted = false; // TODO get the actual inversion
      motorSettings.pid = new PIDController(0, 0, 0);

      FlywheelSettings flywheelSettings = new FlywheelSettings();
      flywheelSettings.color = new Color8Bit(Color.kBlack);
      flywheelSettings.feedforward = new SimpleMotorFeedforward(0, 0);
      flywheelSettings.moiKgMetersSquared = 0.001;
      flywheelSettings.motor = DCMotor.getNeo550(1);

      SparkMaxSettings sparkMaxSettings = new SparkMaxSettings();
      sparkMaxSettings.canId = 30;

      algaeFlywheel =
          new FlywheelMotorSubsystem(
              new MotorIOSparkMax(motorSettings, sparkMaxSettings), "Algae", flywheelSettings);
    }
  }
}
