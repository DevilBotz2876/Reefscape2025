package frc.robot.subsystems.implementations.gizmo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.interfaces.MotorIO;
import frc.robot.io.interfaces.MotorIOInputsAutoLogged;
import frc.robot.subsystems.interfaces.Gizmo;
import org.littletonrobotics.junction.Logger;

/**
 * This GizmoSubsystem extends the WPILib SubsystemBase class and implements the Gizmo interface/API
 */
public class GizmoSubsystem extends SubsystemBase implements Gizmo {
  /* Hardware IO */
  /* This subsystem implements the gizmo using a single instance of a MotorIO object */
  MotorIO io;
  /* This is the logged status that is published to the network tables that can be saved/viewed in Shuffleboard/AdvantageScope/etc. */
  MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

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
    // When extending a class and overriding a method, we generally want the parent class
    // (SubsystemBase) method to be executed and then run our code.  So, here, we call the parent
    // (or "super") version of the periodic function
    super.periodic();

    // Here, we ask the IO implementation to update the status and sensor readings and then capture
    // the info into a log for post-analysis (if necessary)
    io.updateInputs(inputs);
    Logger.processInputs("Gizmo", inputs);

    /* Update 2D Sim */
    // We get the current rotation of the motor
    double currentPositionDegrees = Units.radiansToDegrees(inputs.positionRad);
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
}
