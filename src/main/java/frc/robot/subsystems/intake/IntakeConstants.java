package frc.robot.subsystems.intake;

import frc.robot.Constants;

public class IntakeConstants {
  public static final int angleMotorId = 9;
  public static final int rollersMotorId = 10;
  public static final int handoffMotorId = 11;

  public static final double kLoweredAngle = 26;
  /** in rotations */
  public static final double kRaisedAngle = 0;
  /** in rotations */
  public static final double kClimbingAngle = 5;
  /** in rotations */
  public static final double kRollersSpeed = 1.0;
  /** duty cycle */
  public static final double kIndexerSpeed = 1.0;
  /** duty cycle */
  public static final Gains gains =
      switch (Constants.currentMode) {
        default -> new Gains(90.0);
        case SIM -> new Gains(75.0);
      };

  public record Gains(double kP) {}
}
