package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * A gizmo is a made up mechanism that consists of the following:
 *
 * <p>A pinwheel connected to a motor that has an encoder
 */
public interface Gizmo {
  /* These are global constants that can be customized in each robot configuration that instantiates this subsystem */
  public static class Constants {
    public static int numFins = 5; // Sets the number of fins on the gizmo
    public static int sim2dSize = 30; // Sets the dimension of the 2D sim graphic
  }

  /* 2D Simulation */
  public Mechanism2d mech2d =
      new Mechanism2d(
          Constants.sim2dSize,
          Constants.sim2dSize); /* The top-level structure for the 2D mechanism */
  public List<MechanismLigament2d> pinwheelFins =
      new ArrayList<
          MechanismLigament2d>(); /* contains the pinwheel fin objects so that the positions can be updated based on the current position */
  public static final Color8Bit finColors[] = {
    /* array of possible fin colors */
    new Color8Bit(Color.kRed),
    new Color8Bit(Color.kOrange),
    new Color8Bit(Color.kYellow),
    new Color8Bit(Color.kGreen),
    new Color8Bit(Color.kBlue),
    new Color8Bit(Color.kIndigo),
    new Color8Bit(Color.kViolet)
  };

  /**
   * Sets the speed and rotation of the pinwheel
   *
   * @param speed In the range [-1.0 .. 1.0]. 0.0 is stopped with incre. negative value indicates
   *     counter-clockwise rotation (CCW), positive values indicate clockwise (CW).
   */
  public void setSpeed(double speed);

  /**
   * @return 2D graphical simulation of the pinwheel
   */
  public default Optional<Mechanism2d> create2dSim() {
    /* We create a simulated version of each gizmo fin equally around the point so that it looks line fins of a fan/pinwheel */
    if (0 == pinwheelFins.size()) {
      // We create a pivot point where the gizmo fins are attached
      MechanismRoot2d pinWheelPivot2d =
          mech2d.getRoot("Gizmo Pivot", Constants.sim2dSize / 2, Constants.sim2dSize / 2);

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
    }

    return Optional.of(mech2d);
  }

  public default void update2dSim(double positionDegrees) {
    int angleOffset = 0;

    // We loop through each fin and update the angle to match the reported motor rotation
    for (MechanismLigament2d fin : pinwheelFins) {
      fin.setAngle(angleOffset + positionDegrees);
      angleOffset += 360 / Constants.numFins;
    }
  }
}
