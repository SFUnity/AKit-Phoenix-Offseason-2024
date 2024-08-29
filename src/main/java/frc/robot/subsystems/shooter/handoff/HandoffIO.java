package frc.robot.subsystems.shooter.handoff;

import org.littletonrobotics.junction.AutoLog;

public interface HandoffIO {
  @AutoLog
  public static class HandoffIOInputs {
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(HandoffIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  default void runVolts(double volts) {}

  /** Stop handoff */
  default void stop() {}
}
