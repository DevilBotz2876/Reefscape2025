package frc.robot.config.game.reefscape2025;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.io.implementations.motor.MotorIOArmStub;
import frc.robot.io.implementations.motor.MotorIOBase.MotorIOBaseSettings;
import frc.robot.io.implementations.motor.MotorIOElevatorStub;
import frc.robot.subsystems.controls.arm.ClimberArmControls;
import frc.robot.subsystems.controls.arm.CoralArmControls;
import frc.robot.subsystems.controls.combination.DriverAssistControls;
import frc.robot.subsystems.controls.combination.DriverControls;
import frc.robot.subsystems.controls.drive.DriveControls;
import frc.robot.subsystems.controls.elevator.ElevatorControls;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.motor.ArmMotorSubsystem;
import frc.robot.subsystems.implementations.motor.ElevatorMotorSubsystem;
import frc.robot.subsystems.implementations.vision.VisionSubsystem;
import frc.robot.subsystems.interfaces.Arm.ArmSettings;
import frc.robot.subsystems.interfaces.Elevator.ElevatorSettings;
import frc.robot.subsystems.interfaces.Vision.Camera;

/* Put all constants here with reasonable defaults */
public class RobotConfig {
  public static DriveBase drive;
  public static SendableChooser<Command> autoChooser;
  public static VisionSubsystem vision;
  protected static ElevatorMotorSubsystem elevator;
  public static ArmMotorSubsystem coralArm;
  public static ArmMotorSubsystem climberArm;

  // Controls
  public CommandXboxController mainController = new CommandXboxController(0);
  public CommandXboxController assistController = new CommandXboxController(1);

  public RobotConfig(boolean stubDrive, boolean stubAuto, boolean stubVision) {
    this(stubDrive, stubAuto, stubVision, true, true, true);
  }

  public RobotConfig(
      boolean stubDrive,
      boolean stubAuto,
      boolean stubVision,
      boolean stubElevator,
      boolean stubCoralArm) {
    this(stubDrive, stubAuto, stubVision, stubElevator, stubCoralArm, true);
  }

  public RobotConfig() {
    this(true, true, true, true, true, true);
  }

  public RobotConfig(
      boolean stubDrive,
      boolean stubAuto,
      boolean stubVision,
      boolean stubElevator,
      boolean stubCoralArm,
      boolean stubAlgaeSubsystem) {
    this(stubDrive, stubAuto, stubVision, stubElevator, stubCoralArm, stubAlgaeSubsystem, true);
  }

  public RobotConfig(
      boolean stubDrive,
      boolean stubAuto,
      boolean stubVision,
      boolean stubElevator,
      boolean stubCoralArm,
      boolean stubAlgaeSubsystem,
      boolean stubClimberArm) {
    if (stubDrive) {
      drive = new DriveBase("Stub");
    }

    if (stubAuto) {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("No Auto Routines Specified", Commands.none());
    }

    vision =
        new VisionSubsystem(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));

    if (stubVision) {
      if (Robot.isSimulation()) {
        vision.addCamera(
            new Camera(
                "photonvision",
                new Transform3d(
                    new Translation3d(-0.221, 0, .164),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)))));
        vision.addCamera(
            new Camera(
                "left",
                new Transform3d(
                    new Translation3d(0, 0.221, .164),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(90)))));
        vision.addCamera(
            new Camera(
                "right",
                new Transform3d(
                    new Translation3d(0, -0.221, .164),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-90)))));
      }
    }

    if (stubElevator) {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      motorSettings.motor.gearing = 9;
      motorSettings.motor.inverted = false;
      motorSettings.motor.drumRadiusMeters = Units.inchesToMeters(0.75 / 2); // 3/4" diameter
      motorSettings.pid = new PIDController(1.0, 0, 0);

      ElevatorSettings elevatorSettings = new ElevatorSettings();
      elevatorSettings.minHeightInMeters = 0.0;
      elevatorSettings.maxHeightInMeters = 0.02 + 0.85 + 0.76;
      elevatorSettings.startingHeightInMeters = elevatorSettings.minHeightInMeters;
      elevatorSettings.targetHeightToleranceInMeters = 0.01;
      elevatorSettings.color = new Color8Bit(Color.kSilver);
      elevatorSettings.feedforward = new ElevatorFeedforward(0, 0.0351, 0.17095, 0.0);
      elevatorSettings.carriageMassKg = 2.0;
      elevatorSettings.motor = DCMotor.getKrakenX60(1);
      elevatorSettings.simulateGravity = true;
      elevatorSettings.maxVelocityInMetersPerSecond = 1;
      elevatorSettings.maxAccelerationInMetersPerSecondSquared = 1;

      ElevatorControls.Constants.autoZeroSettings.voltage = 2.0;
      ElevatorControls.Constants.autoZeroSettings.minResetCurrent = 40;
      ElevatorControls.Constants.autoZeroSettings.resetPositionRad =
          elevatorSettings.maxHeightInMeters / motorSettings.motor.drumRadiusMeters;
      ElevatorControls.Constants.autoZeroSettings.initialReverseDuration =
          0.0; // Set the seconds of reverse before zero. Set to zero if there shound be no reverse

      elevator =
          new ElevatorMotorSubsystem(
              new MotorIOElevatorStub(motorSettings, elevatorSettings), "Coral", elevatorSettings);
    }

    if (stubCoralArm) {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      motorSettings.motor.gearing = 50;
      motorSettings.motor.inverted = false;
      motorSettings.pid = new PIDController(1.0, 0, 0);

      ArmSettings armSettings = new ArmSettings();
      armSettings.minAngleInDegrees = -90;
      armSettings.maxAngleInDegrees = 75;
      armSettings.startingAngleInDegrees = armSettings.minAngleInDegrees;
      armSettings.feedforward = new ArmFeedforward(0, 0.222, 0.001, 0);
      armSettings.color = new Color8Bit(Color.kBlue);
      armSettings.armLengthInMeters = 0.5;
      armSettings.armMassInKg = 0.81;
      armSettings.motor = DCMotor.getNEO(1);
      armSettings.simulateGravity = true;

      coralArm =
          new ArmMotorSubsystem(
              new MotorIOArmStub(motorSettings, armSettings), "Coral", armSettings);
    }

    if (stubAlgaeSubsystem) {}

    if (stubClimberArm) {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      motorSettings.motor.gearing = 50;
      motorSettings.motor.inverted = false;
      motorSettings.pid = new PIDController(0, 0, 0);
      motorSettings.reverseLimitChannel = 1;
      motorSettings.reverseLimitNegate = true;

      ArmSettings armSettings = new ArmSettings();
      armSettings.minAngleInDegrees = 0;
      armSettings.maxAngleInDegrees = 135;
      armSettings.startingAngleInDegrees = 90;
      armSettings.color = new Color8Bit(Color.kRed);
      armSettings.feedforward = new ArmFeedforward(0, 0, 0, 0);
      armSettings.armLengthInMeters = 0.5;
      armSettings.armMassInKg = 0.75;
      armSettings.motor = DCMotor.getNEO(1);
      armSettings.simulateGravity = true;

      ClimberArmControls.Constants.autoZeroSettings.voltage = -1.0;
      ClimberArmControls.Constants.autoZeroSettings.minResetCurrent = 40.0;
      ClimberArmControls.Constants.autoZeroSettings.resetPositionRad =
          Units.degreesToRadians(armSettings.minAngleInDegrees);

      climberArm =
          new ArmMotorSubsystem(
              new MotorIOArmStub(motorSettings, armSettings), "Climber", armSettings);
    }
  }

  public void configureBindings() {
    if (Robot.isSimulation()) {
      vision.enableSimulation(() -> RobotConfig.drive.getPose(), true);
    }

    // Send vision-based odometry measurements to drive's odometry calculations
    vision.setVisionMeasurementConsumer(drive::addVisionMeasurement);

    DriveControls.setupController(drive, elevator, coralArm, mainController);
    DriverAssistControls.setupController(elevator, coralArm, assistController);
    DriverControls.setupController(elevator, coralArm, mainController);
    CoralArmControls.setupController(coralArm, mainController);
    ElevatorControls.setupController(elevator, mainController);
    ClimberArmControls.setupController(climberArm, mainController);

    if (null != RobotConfig.autoChooser) {
      SmartDashboard.putData("Autonomous", RobotConfig.autoChooser);
    }
  }
}
