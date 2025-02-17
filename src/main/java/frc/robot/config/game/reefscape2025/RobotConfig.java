package frc.robot.config.game.reefscape2025;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.io.implementations.arm.ArmIOStub;
import frc.robot.io.implementations.elevator.ElevatorIOStub;
import frc.robot.io.implementations.intake.IntakeIOStub;
import frc.robot.io.implementations.motor.MotorIOArmStub;
import frc.robot.io.implementations.motor.MotorIOBase.MotorIOBaseSettings;
import frc.robot.subsystems.controls.algae.AlgaeControls;
import frc.robot.subsystems.controls.arm.ClimberArmControls;
import frc.robot.subsystems.controls.arm.CoralArmControls;
import frc.robot.subsystems.controls.drive.DriveControls;
import frc.robot.subsystems.controls.elevator.ElevatorControls;
import frc.robot.subsystems.controls.vision.VisionControls;
import frc.robot.subsystems.implementations.algae.AlgaeSubsystem;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.elevator.ElevatorSubsystem;
import frc.robot.subsystems.implementations.motor.ArmMotorSubsystem;
import frc.robot.subsystems.implementations.vision.VisionSubsystem;
import frc.robot.subsystems.interfaces.Algae;
import frc.robot.subsystems.interfaces.ArmV2.ArmSettings;
import frc.robot.subsystems.interfaces.Vision.Camera;
import java.util.ArrayList;

/* Put all constants here with reasonable defaults */
public class RobotConfig {
  public static DriveBase drive;
  public static SendableChooser<Command> autoChooser;
  public static VisionSubsystem vision;
  public static ElevatorSubsystem elevator;
  public static ArmMotorSubsystem coralArm;
  public static ArmMotorSubsystem climberArm;
  public static AlgaeSubsystem algaeSubsystem;

  // Controls
  public CommandXboxController mainController = new CommandXboxController(0);
  public CommandXboxController assistController = new CommandXboxController(1);
  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

  // private final ShuffleboardTab pitTab = Shuffleboard.getTab("Pit");
  // private final ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
  // private final ShuffleboardTab sysIdTestTab = Shuffleboard.getTab("SysId");

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
      drive = new DriveBase();
    }

    if (stubAuto) {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("No Auto Routines Specified", Commands.none());
    }

    vision = new VisionSubsystem(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape));

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
      elevator = new ElevatorSubsystem(new ElevatorIOStub());
    }

    if (stubCoralArm) {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      motorSettings.motor.gearing = 50;
      motorSettings.motor.inverted = false;
      motorSettings.pid = new PIDController(1.0, 0, 0);

      ArmSettings armSettings = new ArmSettings();
      armSettings.minAngleInDegrees = 0;
      armSettings.maxAngleInDegrees = 150;
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

    if (stubAlgaeSubsystem) {
      algaeSubsystem =
          new AlgaeSubsystem(
              new IntakeIOStub(),
              new ArmIOStub(
                  Algae.Constants.maxArmAngleDegrees, Algae.Constants.minArmAngleDegrees));
    }

    if (stubClimberArm) {
      MotorIOBaseSettings motorSettings = new MotorIOBaseSettings();
      motorSettings.motor.gearing = 50;
      motorSettings.motor.inverted = false;
      motorSettings.pid = new PIDController(1, 0, 0);
      motorSettings.minLimitChannel = 1;
      motorSettings.minLimitNegate = true;

      ArmSettings armSettings = new ArmSettings();
      armSettings.minAngleInDegrees = 0;
      armSettings.maxAngleInDegrees = 135;
      armSettings.startingAngleInDegrees = armSettings.minAngleInDegrees;
      armSettings.color = new Color8Bit(Color.kRed);
      armSettings.feedforward = new ArmFeedforward(0.0021633, 0.060731, 0.9481, 0);
      armSettings.armLengthInMeters = 0.5;
      armSettings.armMassInKg = 0.75;
      armSettings.motor = DCMotor.getNEO(1);
      armSettings.simulateGravity = true;

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

    DriveControls.setupController(drive, mainController);
    DriveControls.addGUI(drive, driverTab);

    VisionControls.addGUI(vision, driverTab);

    CoralArmControls.setupController(coralArm, mainController);
    ClimberArmControls.setupController(climberArm, mainController);

    // Old subsystems to be replaced
    ElevatorControls.setupController(elevator, mainController);
    ElevatorControls.addSysId(elevator);
    AlgaeControls.setupController(algaeSubsystem, mainController);

    setupSimGUI();
  }

  public void setupSimGUI() {
    Mechanism2d mech2d = new Mechanism2d(60, 60);
    MechanismRoot2d coralRoot = mech2d.getRoot("coral", 0, 0);

    MechanismRoot2d algaeRoot = mech2d.getRoot("algae", 20, 0);

    MechanismLigament2d elevatorLigament2d =
        coralRoot.append(
            new MechanismLigament2d("Elevator", 5, 90, 10, new Color8Bit(Color.kLightSlateGray)));
    elevator.setLigament(elevatorLigament2d);

    MechanismLigament2d algaeArmLigament2d =
        algaeRoot.append(
            new MechanismLigament2d("Algae Arm", 10, 90, 6, new Color8Bit(Color.kOrange)));

    ArrayList<MechanismLigament2d> intakeLigaments2d = new ArrayList<MechanismLigament2d>();
    intakeLigaments2d.add(
        algaeArmLigament2d.append(
            new MechanismLigament2d("Wheel Spoke A", 2.5, 0, 6, new Color8Bit(Color.kGray))));
    intakeLigaments2d.add(
        algaeArmLigament2d.append(
            new MechanismLigament2d("Wheel Spoke B", 2.5, 90, 6, new Color8Bit(Color.kRed))));
    intakeLigaments2d.add(
        algaeArmLigament2d.append(
            new MechanismLigament2d("Wheel Spoke C", 2.5, 180, 6, new Color8Bit(Color.kGray))));
    intakeLigaments2d.add(
        algaeArmLigament2d.append(
            new MechanismLigament2d("Wheel Spoke D", 2.5, 270, 6, new Color8Bit(Color.kRed))));

    algaeSubsystem.setLigament(algaeArmLigament2d, intakeLigaments2d);

    SmartDashboard.putData("2D Simulation", mech2d);
  }
}
