package frc.robot.config.game.reefscape2025;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.io.implementations.motor.MotorIOBase.MotorIOBaseSettings;
import frc.robot.io.implementations.motor.MotorIOFlywheelStub;
import frc.robot.subsystems.controls.flywheel.FlywheelControls;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.implementations.motor.FlywheelMotorSubsystem;
import frc.robot.subsystems.interfaces.Flywheel.FlywheelSettings;

/* Override Phoenix specific constants here */
public class RobotConfigStub extends RobotConfig {
  private final FlywheelMotorSubsystem algaeIntake;

  public RobotConfigStub() {
    super(false, true, false);

    drive = new DriveSwerveYAGSL("yagsl/stub");
    if (Robot.isSimulation()) {
      drive.setPose(new Pose2d(new Translation2d(1, 1), new Rotation2d()));
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

    FlywheelControls.setupController(algaeIntake, mainController);
  }
}
