package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.util.Color;

public interface LED {
  public default Color getColor() {
    return new Color(0, 0, 0);
  }

  public default void setColor(Color color) {}

public void startRainbow(int i, int j, int k);
}
