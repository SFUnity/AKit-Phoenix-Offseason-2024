package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double pivotPositionRad = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double[] pivotCurrentAmps = new double[] {};

    public double rollersAppliedVolts = 0.0;
    public double[] rollersCurrentAmps = new double[] {};

    public double indexerAppliedVolts = 0.0;
    public double[] indexerCurrentAmps = new double[] {};
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
