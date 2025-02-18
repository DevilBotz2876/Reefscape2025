package frc.robot.config.game.reefscape2025;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.io.implementations.motor.MotorIOArmStub;
import frc.robot.io.implementations.motor.MotorIOBase.MotorIOBaseSettings;
import frc.robot.io.implementations.motor.MotorIOElevatorStub;
import frc.robot.io.implementations.motor.MotorIOFlywheelStub;
import frc.robot.subsystems.controls.arm.AlgaeArmControls;
import frc.robot.subsystems.controls.arm.ClimberArmControls;
import frc.robot.subsystems.controls.arm.CoralArmControls;
import frc.robot.subsystems.controls.elevator.ElevatorControlsV2;
import frc.robot.subsystems.controls.flywheel.FlywheelControls;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.implementations.motor.ArmMotorSubsystem;
import frc.robot.subsystems.implementations.motor.ElevatorMotorSubsystem;
import frc.robot.subsystems.implementations.motor.FlywheelMotorSubsystem;
import frc.robot.subsystems.interfaces.ArmV2.ArmSettings;
import frc.robot.subsystems.interfaces.ElevatorV2.ElevatorSettings;
import frc.robot.subsystems.interfaces.Flywheel.FlywheelSettings;

/* Override Phoenix specific constants here */
public class RobotConfigStub extends RobotConfig {
  private final ArmMotorSubsystem algaeArm;
  private final ArmMotorSubsystem climberArm;
  private final ElevatorMotorSubsystem elevator;
  private final FlywheelMotorSubsystem algaeIntake;

  public RobotConfigStub() {
    super(false, true, false);

    drive = new DriveSwerveYAGSL("yagsl/stub");
    if (Robot.isSimulation()) {
      drive.setPose(new Pose2d(new Translation2d(1, 1), new Rotation2d()));
    }

    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      motorSettings.motor.gearing = 50;
      motorSettings.motor.inverted = false;
      motorSettings.pid = new PIDController(0, 0, 0);

      ArmSettings armSettings = new ArmSettings();
      armSettings.minAngleInDegrees = -15;
      armSettings.maxAngleInDegrees = 105;
      armSettings.startingAngleInDegrees = armSettings.maxAngleInDegrees;
      armSettings.color = new Color8Bit(Color.kGreen);
      armSettings.feedforward = new ArmFeedforward(0, 0, 0, 0);
      armSettings.armLengthInMeters = 0.75;
      armSettings.armMassInKg = 1.0;
      armSettings.motor = DCMotor.getKrakenX60(1);
      armSettings.simulateGravity = false;

      algaeArm =
          new ArmMotorSubsystem(
              new MotorIOArmStub(motorSettings, armSettings), "Algae", armSettings);
    }

    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      motorSettings.motor.gearing = 50;
      motorSettings.motor.inverted = false;
      motorSettings.pid = new PIDController(1, 0, 0);

      ArmSettings armSettings = new ArmSettings();
      armSettings.minAngleInDegrees = 0;
      armSettings.maxAngleInDegrees = 90;
      armSettings.startingAngleInDegrees = armSettings.maxAngleInDegrees;
      armSettings.color = new Color8Bit(Color.kRed);
      armSettings.feedforward = new ArmFeedforward(0.0021633, 0.060731, 0.9481, 0);
      armSettings.armLengthInMeters = 0.5;
      armSettings.armMassInKg = 0.75;
      armSettings.motor = DCMotor.getKrakenX60(1);
      armSettings.simulateGravity = true;

      climberArm =
          new ArmMotorSubsystem(
              new MotorIOArmStub(motorSettings, armSettings), "Climber", armSettings);
    }

    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      motorSettings.motor.gearing = 50;
      motorSettings.motor.inverted = false;
      motorSettings.motor.drumRadiusMeters = 0.050; // 50 mm radius
      motorSettings.pid = new PIDController(0, 0, 0);

      ElevatorSettings elevatorSettings = new ElevatorSettings();
      elevatorSettings.minHeightInMeters = 0.1;
      elevatorSettings.maxHeightInMeters = 1.0;
      elevatorSettings.startingHeightInMeters = elevatorSettings.minHeightInMeters;
      elevatorSettings.color = new Color8Bit(Color.kSilver);
      elevatorSettings.feedforward = new ElevatorFeedforward(0, 0, 0, 0);
      elevatorSettings.carriageMassKg = 5.0;
      elevatorSettings.motor = DCMotor.getKrakenX60(1);
      elevatorSettings.simulateGravity = true;

      elevator =
          new ElevatorMotorSubsystem(
              new MotorIOElevatorStub(motorSettings, elevatorSettings), "Coral", elevatorSettings);
    }

    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      motorSettings.motor.gearing = 1;
      motorSettings.motor.inverted = false;
      motorSettings.pid = new PIDController(0, 0, 0);

      FlywheelSettings flywheelSettings = new FlywheelSettings();
      flywheelSettings.color = new Color8Bit(Color.kPurple);
      flywheelSettings.feedforward = new SimpleMotorFeedforward(0, 0);
      flywheelSettings.moiKgMetersSquared = 0.001;
      flywheelSettings.motor = DCMotor.getNeo550(1);

      algaeIntake =
          new FlywheelMotorSubsystem(
              new MotorIOFlywheelStub(motorSettings, flywheelSettings),
              // new MotorIOStub(settings, simulationSettings),
              "Algae",
              flywheelSettings);
    }
  }

  @Override
  public void configureBindings() {
    // Configure the default bindings of the parent class
    super.configureBindings();

    CoralArmControls.setupController(coralArm, mainController);
    AlgaeArmControls.setupController(algaeArm, mainController);
    ClimberArmControls.setupController(climberArm, mainController);
    ElevatorControlsV2.setupController(elevator, mainController);
    FlywheelControls.setupController(algaeIntake, mainController);
  }
}
