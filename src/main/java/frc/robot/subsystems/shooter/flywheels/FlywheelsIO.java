package frc.robot.subsystems.shooter.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
  @AutoLog
  public static class FlywheelsIOInputs {
    public double velocityRadPerSecTop = 0.0;
    public double appliedVoltsTop = 0.0;
    public double[] currentAmpsTop = new double[] {};

    public double velocityRadPerSecBottom = 0.0;
    public double appliedVoltsBottom = 0.0;
    public double[] currentAmpsBottom = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelsIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltageTop(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocityTop(double velocityRadPerSec, double ffVolts) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltageBottom(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocityBottom(double velocityRadPerSec, double ffVolts) {}

  /** Stop in open loop. */
  public default void stopBoth() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
