package frc.robot.subsystems.implementations.motor;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.interfaces.MotorIO;
import frc.robot.subsystems.interfaces.ArmV2;
import org.littletonrobotics.junction.Logger;

public class ArmMotorSubsystem extends MotorSubsystem implements ArmV2 {
  private final ArmSettings settings;
  private double targetAngleDegrees = 0;
  private double targetAngleRad = 0;

  /* 2D Simulation */
  private final Mechanism2d mech2d;
  private final MechanismLigament2d armSegment;

  public ArmMotorSubsystem(MotorIO io, String name, ArmSettings settings) {
    super(io, "Arm[" + name + "]");
    this.settings = settings;

    double sim2dSize = settings.armLengthInMeters * 64 * 2;
    mech2d = new Mechanism2d(sim2dSize, sim2dSize);

    MechanismRoot2d armPivot2d = mech2d.getRoot(getName() + "Pivot", sim2dSize / 2, sim2dSize / 2);

    armSegment =
        armPivot2d.append(
            new MechanismLigament2d(
                getName(), sim2dSize / 2, settings.startingAngleInDegrees, 15, settings.color));

    SmartDashboard.putData(getName() + "/2D Simulation", mech2d);
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(getName() + "/targetAngleRad", targetAngleRad);
    Logger.recordOutput(getName() + "/targetAngleDegrees", targetAngleDegrees);

    // We get the current angle of the motor and update the 2D sim accordingly
    armSegment.setAngle(Units.radiansToDegrees(inputs.positionRad));
  }

  @Override
  public ArmSettings getSettings() {
    return settings;
  }

  @Override
  public double getCurrentAngle() {
    return Units.radiansToDegrees(inputs.positionRad);
  }

  @Override
  public void setTargetAngle(double degrees) {
    this.targetAngleDegrees = degrees;
    this.targetAngleRad = Units.degreesToRadians(targetAngleDegrees);
    io.setPosition(
        this.targetAngleRad,
        settings.feedforward.calculate(
            this.targetAngleRad, Units.degreesToRadians(settings.maxVelocityInDegreesPerSecond)));
  }

  @Override
  public double getTargetAngle() {
    return this.targetAngleDegrees;
  }

  @Override
  public void enableOpenLoop() {
    io.setVoltage(0);
  }

  @Override
  public boolean isAtMaxLimit() {
    return (getTargetAngle() >= settings.maxAngleInDegrees);
  }

  @Override
  public boolean isAtMinLimit() {
    return (getTargetAngle() <= settings.minAngleInDegrees);
  }

  @Override
  public boolean isAtSetpoint() {
    return (Math.abs(getCurrentAngle() - this.targetAngleDegrees)
        <= settings.targetAngleToleranceInDegrees);
  }
}
