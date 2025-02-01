package frc.robot.subsystems.interfaces;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Algae extends Subsystem {
  public static class Constants {
      public static double maxArmAngleDegrees = 90.0;
      public static double minArmAngleDegrees = 0.0;
      public static double defaultIntakeSpeedInVolts = 6.0;

      public static double pidKp = 0.1;
      public static double pidKi = 0.0;
      public static double pidKd = 0.0;
      public static double pidTimeoutInSeconds = 3.0;
  
      public static double ffKs = 0.0;
      public static double ffKv = 0.0;
      public static double ffKa = 0.0;
      public static double ffKg = 0.1;
  }

  // Arm
  public double getArmAngle();

  public double getArmTargetAngle();

  public boolean isArmAtMaxLimit();

  public boolean isArmAtMinLimit();

  public double getArmVelocity();

  // sets of the angle of the arm
  public void setArmAngle(double degrees);

  public boolean isArmAtSetpoint();

  public default void runVoltageArm(double volts) {}


//   public default boolean isPieceDetected() {
//     return false;
//   }

  // Intake
  public default void runVoltageIntake(double volts) {}

  public default double getCurrentVoltageIntake() {
    return 0;
  }

  public default void turnOffIntake() {
    runVoltageIntake(0);
  }

  public Command getTurnOffIntakeCommand();

  public default void turnOnRightIntake() {
    runVoltageIntake(Constants.defaultIntakeSpeedInVolts);
  }

  public default void turnOnLeftIntake() {
    runVoltageIntake(-Constants.defaultIntakeSpeedInVolts);
  }

  public Command getTurnRightIntakeCommand();
  public Command getTurnLeftIntakeCommand();

  public default void setLigament(MechanismLigament2d armLigament2d, ArrayList<MechanismLigament2d> intakeLigament2d) {}
}
