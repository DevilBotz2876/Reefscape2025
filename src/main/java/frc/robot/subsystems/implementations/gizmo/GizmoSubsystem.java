package frc.robot.subsystems.implementations.gizmo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.interfaces.MotorIO;
import frc.robot.io.interfaces.MotorIOInputsAutoLogged;
import frc.robot.subsystems.interfaces.Gizmo;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * This GizmoSubsystem extends the WPILib SubsystemBase class and implements the Gizmo interface/API
 */
public class GizmoSubsystem extends SubsystemBase implements Gizmo {
  /* These are global constants that can be customized in each robot configuration that instantiates this subsystem */
  public static class Constants {
    public static int numFins = 5; // Sets the number of fins on the gizmo
    public static int sim2dSize = 30; // Sets the dimension of the 2D sim graphic
  }

  /* Hardware IO */
  MotorIO io; // This subsystem implements the gizmo using a single instance of a MotorIO object
  MotorIOInputsAutoLogged inputs =
      new MotorIOInputsAutoLogged(); // This is the logged status that is published to the network
  // tables that can be saved/viewed in
  // Shuffleboard/AdvantageScope/etc.

  /* 2D Simulation */
  private Mechanism2d mech2d; // The top-level structure for the 2D mechanism
  private List<MechanismLigament2d> pinwheelFins =
      new ArrayList<
          MechanismLigament2d>(); // contains the pinwheel fin objects so that the positions can be
  // updated based on the current position
  private static final Color8Bit finColors[] = { // array of possible fin colors.
    new Color8Bit(Color.kRed),
    new Color8Bit(Color.kOrange),
    new Color8Bit(Color.kYellow),
    new Color8Bit(Color.kGreen),
    new Color8Bit(Color.kBlue),
    new Color8Bit(Color.kIndigo),
    new Color8Bit(Color.kViolet)
  };

  /**
   * The GizmoSubsystem implements the Gizmo interface using just a single motor. Here, a MotorIO
   * implementation/object is passed in. The MotorIO object can be a fake motor, a NEO, Kraken, etc.
   * It doesn't matter. E.g. we could have 3 different robot configurations: 1) RobotConfigStub -
   * the software can validate the basic interfaces, functionality, controls using a simululated
   * version of a MotorIO, so would get a *MotorIOStub* object passed in. 2) RobotConfigProto - the
   * initial or "prototype" version of the physical robot might use a "Neo + Sparkmax" motor, so
   * would get a *MotorIOSparkMax* object passed in. 3) RobotConfigComp - the final or "competition"
   * version of the physical robot might use a "Kraken X60" motor, instead, so would get a
   * *MotorIOKraken* object passed in.
   *
   * @param io an object that implements the MotorIO interface/API
   */
  public GizmoSubsystem(MotorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    /* When extending a class and overriding a method, we generally want the parent class (SubsystemBase) method to be executed and then run our code.  So, here, we call the parent (or "super") version of the periodic function */
    super.periodic();

    /* Here, we ask the IO implementation to update the status and sensor readings and then capture the info into a log for post-analysis (if necessary) */
    io.updateInputs(inputs);
    Logger.processInputs("Gizmo", inputs);

    /* Update 2D Sim */
    double currentPositionDegrees =
        Units.radiansToDegrees(inputs.positionRad); // We get the current rotation of the motor
    int angleOffset = 0;
    // We loop through each fin and update the angle to match the reported motor rotation
    for (MechanismLigament2d fin : pinwheelFins) {
      fin.setAngle(angleOffset + currentPositionDegrees);
      angleOffset += 360 / Constants.numFins;
    }
  }

  @Override
  public void setSpeed(double speed) {
    // The robot generally uses 12V, so we scale the requested speed from [-1.0 .. 1.0] to [-12.0 ..
    // 12.0].  We also clamp the min/max speed to 1.0 so we don't override drive the motor.
    io.setVoltage(12 * MathUtil.clamp(speed, -1.0, 1.0));
  }

  @Override
  public Optional<Mechanism2d> create2dSim() {
    mech2d = new Mechanism2d(Constants.sim2dSize, Constants.sim2dSize);

    // We create a pivot point where the gizmo fins are attached
    MechanismRoot2d pinWheelPivot2d =
        mech2d.getRoot("Gizmo Pivot", Constants.sim2dSize / 2, Constants.sim2dSize / 2);

    // We create a simulated version of each gizmo fin equally around the point so that it looks
    // line fins of a fan/pinwheel
    for (int i = 0; i < Constants.numFins; i++) {
      pinwheelFins.add(
          pinWheelPivot2d.append(
              new MechanismLigament2d(
                  "Gizmo Fin " + i,
                  Constants.sim2dSize / 2,
                  i * (360 / Constants.numFins),
                  15,
                  finColors[i % finColors.length])));
    }

    return Optional.of(mech2d);
  }
}
