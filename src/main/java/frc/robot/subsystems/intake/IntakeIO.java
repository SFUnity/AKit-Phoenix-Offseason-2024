package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void runIndexer(double percentOutput) {}

  default void setPivotPosition(double positionRotations) {}

  default void runIntakeRollers(double percentOutput) {}

  /** Set PID values */
  default void setPID(double p) {}

  /** Stops motors */
  default void stop() {}
}
