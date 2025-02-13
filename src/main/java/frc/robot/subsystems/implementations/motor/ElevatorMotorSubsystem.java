package frc.robot.subsystems.implementations.motor;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.io.interfaces.MotorIO;
import frc.robot.subsystems.interfaces.ElevatorV2;
import org.littletonrobotics.junction.Logger;

public class ElevatorMotorSubsystem extends MotorSubsystem implements ElevatorV2 {
  private final ElevatorSettings settings;
  private double targetHeightMeters = 0;

  /* 2D Simulation */
  private final Mechanism2d mech2d;
  private final MechanismLigament2d elevatorSegment;

  public ElevatorMotorSubsystem(MotorIO io, String name, ElevatorSettings settings) {
    super(io, "Elevator[" + name + "]");
    this.settings = settings;

    double sim2dSize = settings.maxHeightInMeters * 64;
    mech2d = new Mechanism2d(sim2dSize, sim2dSize);

    MechanismRoot2d elevatorBaseRoot = mech2d.getRoot(getName() + "Base Root", sim2dSize / 2, 0);

    elevatorBaseRoot.append(
        new MechanismLigament2d(
            getName() + "Base",
            settings.minHeightInMeters * 64,
            90,
            sim2dSize / 4,
            new Color8Bit(Color.kOrange)));

    MechanismRoot2d elevatorSegmentRoot =
        mech2d.getRoot(getName() + "Segment Root", sim2dSize / 2, settings.minHeightInMeters * 64);
    elevatorSegment =
        elevatorSegmentRoot.append(
            new MechanismLigament2d(
                getName() + "Segment 0",
                settings.minHeightInMeters * 64,
                90,
                sim2dSize / 8,
                settings.color));

    SmartDashboard.putData(getName() + "/2D Simulation", mech2d);
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(getName() + "/targetHeightMeters", targetHeightMeters);

    // We get the current linear position of the motor and update the 2D sim accordingly
    elevatorSegment.setLength(inputs.positionMeters * 64);
  }

  @Override
  public ElevatorSettings getSettings() {
    return settings;
  }

  @Override
  public double getCurrentHeight() {
    return Units.radiansToDegrees(inputs.positionRad);
  }

  @Override
  public void setTargetHeight(double meters) {
    this.targetHeightMeters = meters;
    io.setPosition(
        this.targetHeightMeters,
        settings.feedforward.calculate(
            this.targetHeightMeters,
            Units.degreesToRadians(settings.maxVelocityInMetersPerSecond)));
  }

  @Override
  public double getTargetHeight() {
    return this.targetHeightMeters;
  }

  @Override
  public boolean isAtMaxLimit() {
    return (getTargetHeight() >= settings.maxHeightInMeters);
  }

  @Override
  public boolean isAtMinLimit() {
    return (getTargetHeight() <= settings.minHeightInMeters);
  }

  @Override
  public boolean isAtSetpoint() {
    return (Math.abs(getCurrentHeight() - this.targetHeightMeters)
        <= settings.targetHeightToleranceInMeters);
  }
}
