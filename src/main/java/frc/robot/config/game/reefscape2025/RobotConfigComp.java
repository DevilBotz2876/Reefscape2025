package frc.robot.config.game.reefscape2025;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.io.implementations.motor.MotorIOBase.MotorIOBaseSettings;
import frc.robot.io.implementations.motor.MotorIOTalonFx;
import frc.robot.io.implementations.motor.MotorIOTalonFx.TalonFxSettings;
import frc.robot.subsystems.controls.arm.CoralArmControls;
import frc.robot.subsystems.controls.elevator.ElevatorControls;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.implementations.motor.ArmMotorSubsystem;
import frc.robot.subsystems.implementations.motor.ElevatorMotorSubsystem;
import frc.robot.subsystems.interfaces.Arm.ArmSettings;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.subsystems.interfaces.Elevator.ElevatorSettings;

public class RobotConfigComp extends RobotConfig {
  public RobotConfigComp() {
    super(false, true, true, false, false, true, true);

    // Comp has a Swerve drive train
    Drive.Constants.rotatePidKp = 0.025;
    Drive.Constants.rotatePidKi = 0.0;
    Drive.Constants.rotatePidKd = 0.0;
    DriveBase.Constants.rotatePidErrorInDegrees = 1;
    drive = new DriveSwerveYAGSL("yagsl/comp");

    // Elevator
    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      motorSettings.motor.gearing = 9; /* 2x 3:1 gear boxes */
      motorSettings.motor.inverted = true;
      // TODO: Get the DIO ports
      // motorSettings.forwardLimitChannel = 7;
      // motorSettings.forwardLimitNegate = true;
      // motorSettings.reverseLimitChannel = 2;
      // motorSettings.reverseLimitNegate = true;
      motorSettings.motor.drumRadiusMeters =
          Units.inchesToMeters(
              ((1.16 + 0.75) / 2) / 2); // 3/4" inner diameter to 1.16" outer when fully spooled.
      motorSettings.pid = new PIDController(0.2, 0, 0);
      ElevatorSettings elevatorSettings = new ElevatorSettings();
      elevatorSettings.minHeightInMeters = 0.09 + 0.02;
      elevatorSettings.maxHeightInMeters = 0.02 + 0.85 + 0.76;
      elevatorSettings.startingHeightInMeters = elevatorSettings.minHeightInMeters;
      elevatorSettings.color = new Color8Bit(Color.kSilver);
      elevatorSettings.feedforward =
          new ElevatorFeedforward(0.010472, 0.17328, 0.16928, 0.010615); // SysID 2025-02-28
      elevatorSettings.carriageMassKg = 5.0;
      elevatorSettings.motor = DCMotor.getKrakenX60(1);
      elevatorSettings.simulateGravity = true;

      TalonFxSettings talonFxSettings = new TalonFxSettings();
      talonFxSettings.canId = 20;

      elevatorSettings.maxVelocityInMetersPerSecond = 1;
      elevatorSettings.maxAccelerationInMetersPerSecondSquared = 8;

      ElevatorControls.Constants.autoZeroSettings.voltage = 1.5;
      ElevatorControls.Constants.autoZeroSettings.minResetCurrent = 8;
      ElevatorControls.Constants.autoZeroSettings.resetPositionRad =
          elevatorSettings.maxHeightInMeters / motorSettings.motor.drumRadiusMeters;
      ElevatorControls.Constants.autoZeroSettings.initialReverseDuration =
          0.0; // Set the seconds of reverse before zero. Set to zero if there shound be no reverse

      elevator =
          new ElevatorMotorSubsystem(
              new MotorIOTalonFx(motorSettings, talonFxSettings), "Coral", elevatorSettings);
    }

    // Coral Arm
    {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      // 20:1 gear box, 30 teeth on the arm cog and 15 teeth on the motor cog
      motorSettings.motor.gearing = 40;
      motorSettings.motor.inverted = true; // false for Sim
      motorSettings.pid = new PIDController(1.0, 0.0, 0);
      // TODO: Get the DIO ports
      // motorSettings.reverseLimitChannel = 1;
      // motorSettings.reverseLimitNegate = true;

      ArmSettings armSettings = new ArmSettings();
      armSettings.minAngleInDegrees = -90;
      armSettings.maxAngleInDegrees = 60;
      armSettings.startingAngleInDegrees = armSettings.minAngleInDegrees;
      armSettings.feedforward = new ArmFeedforward(0.0, 0.17, 0.7, 0.0);
      armSettings.color = new Color8Bit(Color.kRed);
      armSettings.armLengthInMeters = 0.5;
      armSettings.armMassInKg = 1.0;
      armSettings.motor = DCMotor.getKrakenX60(1);
      armSettings.simulateGravity = true;
      armSettings.maxVelocityInDegreesPerSecond = 180;
      armSettings.maxAccelerationInDegreesPerSecondSquared = 720;

      TalonFxSettings talonFxSettings = new TalonFxSettings();
      talonFxSettings.canId = 21;

      CoralArmControls.Constants.autoZeroSettings.voltage = -0.5;
      CoralArmControls.Constants.autoZeroSettings.minResetCurrent = 0.5;
      CoralArmControls.Constants.autoZeroSettings.resetPositionRad =
          Units.degreesToRadians(
              armSettings.minAngleInDegrees - 10); // We have an offest about 15 degrees
      CoralArmControls.Constants.autoZeroSettings.initialReverseDuration =
          0.0; // Set the seconds of reverse before zero. Set to zero if there shound be no reverse

      coralArm =
          new ArmMotorSubsystem(
              // new MotorIOArmStub(motorSettings, armSettings), "Coral", armSettings);
              new MotorIOTalonFx(motorSettings, talonFxSettings), "Coral", armSettings);
    }

    // climber
    // {
    //     MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
    //     // 25:1 gear box ratio
    //     motorSettings.motor.gearing = 25;
    //     motorSettings.motor.inverted = false; // false for Sim
    //     motorSettings.pid = new PIDController(0.0, 0, 0);
    //     // TODO: Get the DIO ports
    //     // motorSettings.reverseLimitChannel = 1;
    //     // motorSettings.reverseLimitNegate = true;

    //     ArmSettings armSettings = new ArmSettings();
    //     armSettings.minAngleInDegrees = 0;
    //     armSettings.maxAngleInDegrees = 135;
    //     armSettings.startingAngleInDegrees = armSettings.minAngleInDegrees;
    //     armSettings.feedforward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
    //     armSettings.color = new Color8Bit(Color.kRed);
    //     armSettings.armLengthInMeters = 0.5;
    //     armSettings.armMassInKg = 1.0;
    //     armSettings.motor = DCMotor.getNEO(1);
    //     armSettings.simulateGravity = true;

    //     TalonFxSettings settings = new TalonFxSettings();
    //     settings.canId = 50;

    //     ClimberArmControls.Constants.autoZeroSettings.voltage = -1.0;
    //     // Set this to something big, we are never going to use stall current to detect if
    // climber has
    //     // reached it's end of range of motion.
    //     ClimberArmControls.Constants.autoZeroSettings.minResetCurrent = 10.0;
    //     ClimberArmControls.Constants.autoZeroSettings.resetPositionRad =
    //         Units.degreesToRadians(armSettings.minAngleInDegrees);

    //     climberArm =
    //         new ArmMotorSubsystem(
    //             // new MotorIOArmStub(motorSettings, armSettings), "Coral", armSettings);
    //             new MotorIOTalonFx(motorSettings, settings), "Climber", armSettings);
    // }

    CameraServer.startAutomaticCapture();
  }
}
