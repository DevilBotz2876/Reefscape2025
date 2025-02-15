package frc.robot.config.game.reefscape2025;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.io.implementations.motor.MotorIOBase.MotorIOBaseSettings;
import frc.robot.io.implementations.motor.MotorIOTalonFx;
import frc.robot.io.implementations.motor.MotorIOTalonFx.TalonFxSettings;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.implementations.motor.ArmMotorSubsystem;
import frc.robot.subsystems.interfaces.ArmV2.ArmSettings;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Vision.Camera;

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
      motorSettings.pid = new PIDController(0.0, 0, 0);

      ArmSettings armSettings = new ArmSettings();
      armSettings.minAngleInDegrees = 0;
      armSettings.maxAngleInDegrees = 150;
      armSettings.startingAngleInDegrees = armSettings.minAngleInDegrees;
      armSettings.feedforward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
      armSettings.color = new Color8Bit(Color.kRed);
      armSettings.armLengthInMeters = 0.5;
      armSettings.armMassInKg = 1.0;
      armSettings.motor = DCMotor.getKrakenX60(1);
      armSettings.simulateGravity = true;

      TalonFxSettings talonFxSettings = new TalonFxSettings();
      talonFxSettings.canId = 21;

      coralArm =
          new ArmMotorSubsystem(
              // new MotorIOArmStub(motorSettings, armSettings), "Coral", armSettings);
              new MotorIOTalonFx(motorSettings, talonFxSettings), "Coral", armSettings);
    }

    vision.addCamera(
        new Camera(
            "my-first-photonvision", // Back camera
            new Transform3d(
                new Translation3d(-0.3429, 0, 0.0889),
                new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(180)))));
    vision.addCamera(
        new Camera(
            "left_camera",
            new Transform3d(
                new Translation3d(0.0127, 0.3556, 0.0889),
                new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(90)))));
    vision.addCamera(
        new Camera(
            "right_camera",
            new Transform3d(
                new Translation3d(0.13335, -0.3556, 0.0889),
                new Rotation3d(0, Units.degreesToRadians(-5), Units.degreesToRadians(270)))));
    vision.addCamera(
        new Camera(
            "front_camera", // TODO camera too high: place lower on elevator, then fix this tf
            new Transform3d(
                new Translation3d(0.08255, -0.2794, 0.9525),
                new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(10)))));
  }
}
