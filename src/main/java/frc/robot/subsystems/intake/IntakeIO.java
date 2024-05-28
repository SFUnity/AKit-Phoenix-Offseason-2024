package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double pivotPositionRots = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;

    public double rollersAppliedVolts = 0.0;
    public double rollersCurrentAmps = 0.0;

    public double indexerAppliedVolts = 0.0;
    public double indexerCurrentAmps = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void runIndexer(double percentOutput) {}

  default void setPivotPosition(double setpointRads) {}

  default void runIntakeRollers(double percentOutput) {}

  /** Set PID values */
  default void setPID(double p) {}

  /** Stops motors */
  default void stop() {}
}
