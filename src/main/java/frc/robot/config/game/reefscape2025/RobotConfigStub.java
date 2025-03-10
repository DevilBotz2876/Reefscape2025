package frc.robot.config.game.reefscape2025;

import edu.wpi.first.math.controller.ArmFeedforward;
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
import frc.robot.io.implementations.motor.MotorIOFlywheelStub;
import frc.robot.subsystems.controls.arm.AlgaeArmControls;
import frc.robot.subsystems.controls.flywheel.FlywheelControls;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.implementations.motor.ArmMotorSubsystem;
import frc.robot.subsystems.implementations.motor.FlywheelMotorSubsystem;
import frc.robot.subsystems.interfaces.Arm.ArmSettings;
import frc.robot.subsystems.interfaces.Flywheel.FlywheelSettings;

/* Override Phoenix specific constants here */
public class RobotConfigStub extends RobotConfig {
  private final ArmMotorSubsystem algaeArm;
  private final FlywheelMotorSubsystem algaeIntake;

  public RobotConfigStub() {
    super(false, true, true);

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

    AlgaeArmControls.setupController(algaeArm, mainController);
    FlywheelControls.setupController(algaeIntake, mainController);
  }
}
