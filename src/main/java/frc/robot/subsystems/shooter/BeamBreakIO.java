package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
  @AutoLog
  public static class BeamBreakInputs {
    public double distSensorRange;
    public boolean isRangeValid = true;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(BeamBreakInputs inputs) {}
}
