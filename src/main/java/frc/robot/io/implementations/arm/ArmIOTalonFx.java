package frc.robot.io.implementations.arm;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.io.interfaces.ArmIO;
import frc.robot.io.interfaces.ArmIO.ArmIOInputs;
import frc.robot.subsystems.implementations.arm.ArmSubsystem;
import frc.robot.subsystems.interfaces.Arm;

public class ArmIOTalonFx implements ArmIO {
  private final TalonFX motorFx;

  public double kP, kI, kD, kMaxOutput, kMinOutput;
  double rotationsToDegreesConversionFactor;

  public ArmIOTalonFx(int id) {
    this(id, false);
  }

  public ArmIOTalonFx(int id, boolean inverted) {
    motorFx = new TalonFX(id);
    motorFx.setNeutralMode(NeutralModeValue.Brake);
    motorFx.setInverted(inverted); // Deprecated

    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

    /* Configure the Talon to use a supply limit of 70 A, and
     * lower to 40 A if we're at 70 A for over 1 second */

    currentLimitsConfigs
        .withSupplyCurrentLowerLimit(Units.Amps.of(70)) // Default limit of 70 A
        .withSupplyCurrentLimit(
            Units.Amps.of(40)) // Reduce the limit to 40 A if we've limited to 70 A...
        .withSupplyCurrentLowerTime(Units.Seconds.of(1.0)) // ...for at least 1 second
        .withSupplyCurrentLimitEnable(true); // And enable it

    currentLimitsConfigs
        .withStatorCurrentLimit(Units.Amps.of(120)) // Limit stator current to 120 A
        .withStatorCurrentLimitEnable(true); // And enable it

    kP = Arm.Constants.pidKp;
    kI = Arm.Constants.pidKi;
    kD = Arm.Constants.pidKd;
    kMaxOutput = ArmSubsystem.Constants.pidMaxOutput;
    kMinOutput = ArmSubsystem.Constants.pidMinOutput;

    toConfigure.Slot0.kP = kP; // An error of 1 rotation results in 2.4 V output
    toConfigure.Slot0.kI = kI; // No output for integrated error
    toConfigure.Slot0.kD = kD; // A velocity of 1 rps results in 0.1 V output

    // // Peak output of 8 V
    toConfigure
        .Voltage
        .withPeakForwardVoltage(Units.Volts.of(8))
        .withPeakReverseVoltage(Units.Volts.of(-8));

    toConfigure.CurrentLimits = currentLimitsConfigs;
      

    motorFx.getConfigurator().apply(toConfigure);
    motorFx.setPosition(0);

    double gearRatio = 20 * (30.0 / 15.0);
    rotationsToDegreesConversionFactor = 360.0 / gearRatio;

  }

  @Override
  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -12, 12);
    motorFx.setVoltage(volts);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.appliedVolts = motorFx.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motorFx.getSupplyCurrent().getValueAsDouble();
    // inputs.currentStatorAmps = motorFx.getStatorCurrent().getValueAsDouble();
    inputs.positionDegrees = motorFx.getPosition().getValueAsDouble() * rotationsToDegreesConversionFactor;
    inputs.velocityDegrees = motorFx.getVelocity().getValue().in(Units.DegreesPerSecond);

    SmartDashboard.putNumber("Arm/motorFx.getPosition().getValue().in(Units.Degrees);", motorFx.getPosition().getValue().in(Units.Degrees));
    SmartDashboard.putNumber("Arm/motorFx.getPosition().getValue().in(Units.Degree);", motorFx.getPosition().getValue().in(Units.Degree));

    SmartDashboard.putNumber("Arm/motorFx.getPosition().getValue().in(Units.Revolution);", motorFx.getPosition().getValue().in(Units.Revolution));

    SmartDashboard.putNumber("Arm/motorFx.getPosition().getValueAsDouble();", motorFx.getPosition().getValueAsDouble());

    // Code below allows PID to be tuned using SmartDashboard.  And outputs extra data to
    // SmartDashboard.
    if (Constants.debugMode) {
      //   double lp = SmartDashboard.getNumber("Arm/pid/P Gain", 0);
      //   double li = SmartDashboard.getNumber("Arm/pid/I Gain", 0);
      //   double ld = SmartDashboard.getNumber("Arm/pid/D Gain", 0);
      //   double liz = SmartDashboard.getNumber("Arm/pid/I Zone", 0);
      //   double lff = SmartDashboard.getNumber("Arm/pid/Feed Forward", 0);
      //   double lmax = SmartDashboard.getNumber("Arm/pid/Max Output", 0);
      //   double lmin = SmartDashboard.getNumber("Arm/pid/Min Output", 0);

      //   if ((lp != lkP)) {
      //     armPid.setP(lp);
      //     lkP = lp;
      //   }
      //   if ((li != lkI)) {
      //     armPid.setI(li);
      //     lkI = li;
      //   }
      //   if ((ld != lkD)) {
      //     armPid.setD(ld);
      //     lkD = ld;
      //   }
      //   if ((liz != lkIz)) {
      //     armPid.setIZone(liz);
      //     lkIz = liz;
      //   }
      //   if ((lff != lkFF)) {
      //     armPid.setFF(lff);
      //     lkFF = lff;
      //   }
      //   if ((lmax != lkMaxOutput) || (lmin != lkMinOutput)) {
      //     armPid.setOutputRange(lmin, lmax);
      //     lkMinOutput = lmin;
      //     lkMaxOutput = lmax;
      //   }
    }
  }

  public void setPosition(double meters, double ffVolts) {}

  @Override
  public void setFeedback(double kP, double kI, double kD, double minOutput, double maxOutput) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kMinOutput = minOutput;
    this.kMaxOutput = maxOutput;
  }
}
