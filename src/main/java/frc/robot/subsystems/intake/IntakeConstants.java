package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeConstants {
  public static final int angleMotorId = 9;
  public static final int rollersMotorId = 10;
  public static final int handoffMotorId = 11;

  public static final double intakeLength = Units.inchesToMeters(13.835);
  public static final Translation2d intakeOrigin = new Translation2d(.263, .28);

  public static final Gains gains =
      switch (Constants.currentMode) {
        default -> new Gains(0.08);
        case SIM -> new Gains(20);
      };

  public record Gains(double kP) {}
}
