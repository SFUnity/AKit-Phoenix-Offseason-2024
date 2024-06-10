package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeConstants {
  public static final int angleMotorId = 9;
  public static final int rollersMotorId = 10;
  public static final int handoffMotorId = 11;

  /** in rotations */
  public static final double kLoweredAngle = 26;
  /** in rotations */
  public static final double kRaisedAngle = 0;
  /** in rotations */
  public static final double kClimbingAngle = 5;
  /** duty cycle */
  public static final double kRollersSpeed = 1.0;
  /** duty cycle */
  public static final double kIndexerSpeed = 1.0;

  public static final double intakeLength = Units.inchesToMeters(13.835);
  public static final Translation2d intakeOrigin = new Translation2d(0, 0);

  public static final Gains gains =
      switch (Constants.currentMode) {
        default -> new Gains(0.08);
        case SIM -> new Gains(20);
      };

  public record Gains(double kP) {}
}
