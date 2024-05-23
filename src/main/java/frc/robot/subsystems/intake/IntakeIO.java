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

    /** Run to setpoint angle in radians */
    default void runSetpoint(double setpointRads, double feedforward) {}

    /** Run motors at volts */
    default void runVolts(double volts) {}

    /** Set PID values */
    default void setPID(double p, double i, double d) {}

    /** Stops motors */
    default void stop() {}
}
