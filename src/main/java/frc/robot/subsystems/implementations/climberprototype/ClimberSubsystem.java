package frc.robot.subsystems.implementations.climberprototype;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.io.interfaces.MotorControllerIO;
import frc.robot.io.interfaces.MotorControllerIOInputsAutoLogged;
import frc.robot.subsystems.interfaces.ClimberPrototype;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The SingleMotorSubsystem class represents a subsystem with a single motor.
 * It extends the ProfiledPIDSubsystem and implements the SingleMotor interface.
 */
public class ClimberSubsystem extends SubsystemBase implements ClimberPrototype {
  
  /**
   * Constants used in the SingleMotorSubsystem.
   */
  public static class Constants {

  }

  MotorControllerIO io;
  // private final SimpleMotorFeedforward feedforward;
  private final MotorControllerIOInputsAutoLogged inputs = new MotorControllerIOInputsAutoLogged();

  private final SysIdRoutine sysId;
  @AutoLogOutput private double speed;

    /**
   * Constructs a SingleMotorSubsystem with the given MotorControllerIO.
   *
   * @param io the MotorControllerIO instance
   */
  public ClimberSubsystem(MotorControllerIO io) {
    // super(
    //     new ProfiledPIDController(
    //         ClimberPrototype.Constants.pidKp,
    //         ClimberPrototype.Constants.pidKi,
    //         ClimberPrototype.Constants.pidKd,
    //         new TrapezoidProfile.Constraints(
    //             Units.rotationsPerMinuteToRadiansPerSecond(ClimberPrototype.Constants.maxVelocityInRPM),
    //             Units.rotationsPerMinuteToRadiansPerSecond(
    //               ClimberPrototype.Constants.maxAccelerationInRPMSquared))));

    this.io = io;

    // feedforward =
    //     new SimpleMotorFeedforward(
    //         ClimberPrototype.Constants.ffKs, ClimberPrototype.Constants.ffKv, ClimberPrototype.Constants.ffKa);



    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("ClimberPrototype/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setVolts(voltage.in(Volts)), null, this));
  }

  // @Override
  // public void useOutput(double output, TrapezoidProfile.State setpoint) {
  //   // double ff = feedforward.calculate(setpoint.position);

  //   // if (useSoftwarePidVelocityControl) {
  //   //   // Use feedforward +  SW velocity PID
  //   //   io.setVoltage(output + ff);
  //   // } else {
  //   //   // Use feedforward +  HW velocity PID (ignore SW PID)
  //   //   io.setVelocity(setpoint.position, ff);
  //   // }

  //   // currentVelocitySetpointRPM = Units.radiansPerSecondToRotationsPerMinute(setpoint.position);
  //   //    System.out.println(setpoint.position);
  // }

  // @Override
  // public double getMeasurement() {
  //   return inputs.velocityRadPerSec;
  // }

  @Override
  // Sets the voltage to volts. the volts value is -12 to 12
  public void setVolts(double volts) {
    // targetVoltage = volts;
    // targetVelocityRadPerSec = 0;
    // //this.targetVelocityRPM = ClimberPrototype.Constants.maxVelocityInRPM * (volts / 12.0);
    //disable(); // disable PID control
    io.setVoltage(0);
  }



  @Override
  public double getSpeed() {
    return inputs.appliedVolts;
  }


  @Override
  public void periodic() {
    super.periodic();
    // Updates the inputs
    io.updateInputs(inputs);
    Logger.processInputs("Single Motor", inputs);
  }

  public Command getTurnOffCommand() {
    return runOnce(() -> turnOff());
  }

  @Override
  public void add2dSim(Mechanism2d mech2d) {
    // Create 2D simulated display of a Shooter
    MechanismRoot2d intakePivot2d = mech2d.getRoot("Simple Motor", 15, 50);

  }

}
