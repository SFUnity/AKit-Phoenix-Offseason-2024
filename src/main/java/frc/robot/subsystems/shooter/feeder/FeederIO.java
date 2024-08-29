package frc.robot.subsystems.shooter.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(FeederIOInputs inputs) {}

  /** Run open loop at the specified percent output. */
  default void runPercent(double percent) {}

  /** Stop handoff */
  default void stop() {}
}
