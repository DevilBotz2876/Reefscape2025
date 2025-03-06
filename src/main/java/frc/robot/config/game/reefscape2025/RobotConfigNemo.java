package frc.robot.config.game.reefscape2025;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.io.implementations.motor.MotorIOBase.MotorIOBaseSettings;
import frc.robot.io.implementations.motor.MotorIOSparkMax;
import frc.robot.io.implementations.motor.MotorIOSparkMax.SparkMaxSettings;
import frc.robot.io.implementations.motor.MotorIOTalonFx;
import frc.robot.io.implementations.motor.MotorIOTalonFx.TalonFxSettings;
import frc.robot.subsystems.controls.arm.ClimberArmControls;
import frc.robot.subsystems.controls.arm.CoralArmControls;
import frc.robot.subsystems.controls.elevator.ElevatorControls;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.implementations.motor.ArmMotorSubsystem;
import frc.robot.subsystems.implementations.motor.ElevatorMotorSubsystem;
import frc.robot.subsystems.implementations.motor.SimpleMotorSubsystem;
import frc.robot.subsystems.interfaces.Arm.ArmSettings;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Elevator.ElevatorSettings;
import frc.robot.subsystems.interfaces.SimpleMotor.SimpleMotorSettings;
import frc.robot.subsystems.interfaces.Vision.Camera;

/* Override Nemo specific constants here */
public class RobotConfigNemo extends RobotConfig {
  public RobotConfigNemo() {
    super(false, true, true, false, false, true, false);

    // Nemo has a Swerve drive train
    Drive.Constants.rotatePidKp = 0.025;
    Drive.Constants.rotatePidKi = 0.0;
    Drive.Constants.rotatePidKd = 0.0;
    DriveBase.Constants.rotatePidErrorInDegrees = 1;
    drive = new DriveSwerveYAGSL("yagsl/nemo");

    // Camera placement at WPI practice field (2/21/2025)
    vision.addCamera(
        new Camera(
            "my-first-photonvision", // back
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-14.5),
                    Units.inchesToMeters(-3.75),
                    Units.inchesToMeters(7)),
                new Rotation3d(0.0, Units.degreesToRadians(-5), Units.degreesToRadians(180)))));
    vision.addCamera(
        new Camera(
            "left_camera",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-4.4),
                    Units.inchesToMeters(14.5),
                    Units.inchesToMeters(7)),
                new Rotation3d(0.0, Units.degreesToRadians(-5), Units.degreesToRadians(90)))));
    vision.addCamera(
        new Camera(
            "right_camera",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-4.4),
                    Units.inchesToMeters(-14.5),
                    Units.inchesToMeters(7)),
                new Rotation3d(0.0, Units.degreesToRadians(-5), Units.degreesToRadians(270)))));
    vision.addCamera(
        new Camera(
            "front_camera",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(3),
                    Units.inchesToMeters(-10.75),
                    Units.inchesToMeters(36.75)),
                new Rotation3d(0.0, Units.degreesToRadians(30), Units.degreesToRadians(10)))));

    // Coral Arm
    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      // 20:1 gear box, 30 teeth on the arm cog and 15 teeth on the motor cog
      motorSettings.motor.gearing = 20 * (30.0 / 15.0);
      motorSettings.motor.inverted = true; // false for Sim
      motorSettings.pid = new PIDController(1.0, 0.0, 0);

      ArmSettings armSettings = new ArmSettings();
      armSettings.minAngleInDegrees = -90;
      armSettings.maxAngleInDegrees = 60;
      armSettings.startingAngleInDegrees = armSettings.minAngleInDegrees;
      armSettings.feedforward = new ArmFeedforward(0.0, 0.22, 0.0, 0.01);
      armSettings.color = new Color8Bit(Color.kRed);
      armSettings.armLengthInMeters = 0.5;
      armSettings.armMassInKg = 1.0;
      armSettings.motor = DCMotor.getKrakenX60(1);
      armSettings.simulateGravity = true;
      armSettings.maxVelocityInDegreesPerSecond = 360 * 2;
      armSettings.maxAccelerationInDegreesPerSecondSquared = 360;

      TalonFxSettings talonFxSettings = new TalonFxSettings();
      talonFxSettings.canId = 21;

      CoralArmControls.Constants.autoZeroSettings.voltage = -0.5;
      CoralArmControls.Constants.autoZeroSettings.minResetCurrent = 0.5;
      CoralArmControls.Constants.autoZeroSettings.resetPositionRad =
          Units.degreesToRadians(
              armSettings.minAngleInDegrees); // We have an offest about 15 degrees
      CoralArmControls.Constants.autoZeroSettings.initialReverseDuration =
          1.0; // Set the seconds of reverse before zero. Set to zero if there shound be no reverse

      coralArm =
          new ArmMotorSubsystem(
              // new MotorIOArmStub(motorSettings, armSettings), "Coral", armSettings);
              new MotorIOTalonFx(motorSettings, talonFxSettings), "Coral", armSettings);
    }

    // Elevator
    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      motorSettings.motor.gearing = 9; /* 2x 3:1 gear boxes */
      motorSettings.motor.inverted = true;
      // motorSettings.forwardLimitChannel = 7;
      // motorSettings.forwardLimitNegate = true;
      motorSettings.reverseLimitChannel = 2;
      motorSettings.reverseLimitNegate = true;
      motorSettings.motor.drumRadiusMeters =
          Units.inchesToMeters(
              ((1.75 + 0.75) / 2)
                  / 2); // 3/4" inner diameter to 1 3/4" outer. Average diameter calculated For now,
      // use the diameter so that
      // we don't reach the limits
      motorSettings.pid = new PIDController(0.1, 0, 0); // TODO: Tune PID controller

      ElevatorSettings elevatorSettings = new ElevatorSettings();
      elevatorSettings.minHeightInMeters = 0.3;
      elevatorSettings.maxHeightInMeters =
          Units.inchesToMeters(74 - 18); // highest point:74 lowest point:18
      elevatorSettings.startingHeightInMeters = 0.3; // The elevator height when piece is in intake
      elevatorSettings.color = new Color8Bit(Color.kSilver);
      elevatorSettings.feedforward =
          new ElevatorFeedforward(0, 0.34, 0.2, 0); // TODO: Tune feedforward
      elevatorSettings.carriageMassKg = 5.0;
      elevatorSettings.motor = DCMotor.getKrakenX60(1);
      elevatorSettings.simulateGravity = true;

      SparkMaxSettings sparkMaxSettings = new SparkMaxSettings();
      sparkMaxSettings.canId = 20;
      elevatorSettings.maxVelocityInMetersPerSecond = 2.0;
      elevatorSettings.maxAccelerationInMetersPerSecondSquared = 8.0;
      ElevatorControls.Constants.autoZeroSettings.voltage = 1.5;
      ElevatorControls.Constants.autoZeroSettings.minResetCurrent = 30;
      ElevatorControls.Constants.autoZeroSettings.resetPositionRad =
          elevatorSettings.maxHeightInMeters / motorSettings.motor.drumRadiusMeters;
      ElevatorControls.Constants.autoZeroSettings.initialReverseDuration =
          0.0; // Set the seconds of reverse before zero. Set to zero if there shound be no reverse

      elevator =
          new ElevatorMotorSubsystem(
              new MotorIOSparkMax(motorSettings, sparkMaxSettings), "Coral", elevatorSettings);
    }

    // climber
    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      // 25:1 gear box ratio
      motorSettings.motor.gearing = 25;
      motorSettings.motor.inverted = true;
      motorSettings.pid = new PIDController(0.0, 0, 0);
      motorSettings.reverseLimitChannel = 1;
      motorSettings.reverseLimitNegate = true;

      SimpleMotorSettings simpleMotorSettings = new SimpleMotorSettings();
      simpleMotorSettings.minPositionInRads = 0;
      simpleMotorSettings.maxPositionInRads = 14.5;
      simpleMotorSettings.startingPositionInRads = 9;
      simpleMotorSettings.color = new Color8Bit(Color.kRed);
      simpleMotorSettings.feedforward = new SimpleMotorFeedforward(0, 0, 0);
      simpleMotorSettings.motor = DCMotor.getNEO(1);

      SparkMaxSettings settings = new SparkMaxSettings();
      settings.canId = 50;

      ClimberArmControls.Constants.autoZeroSettings.voltage = -1;
      // Set this to something big, we are never going to use stall current to detect if climber has
      // reached it's end of range of motion.
      ClimberArmControls.Constants.autoZeroSettings.minResetCurrent = 20.0;
      ClimberArmControls.Constants.autoZeroSettings.resetPositionRad =
          simpleMotorSettings.minPositionInRads;
      ClimberArmControls.Constants.autoZeroSettings.initialReverseDuration = 0;

      climberArm =
          new SimpleMotorSubsystem(
              new MotorIOSparkMax(motorSettings, settings), "Climber", simpleMotorSettings);
    }
  }
}
