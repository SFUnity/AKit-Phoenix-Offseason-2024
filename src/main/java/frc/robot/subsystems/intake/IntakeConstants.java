package frc.robot.subsystems.intake;

import frc.robot.Constants;

public class IntakeConstants {
  public static final Gains gains =
      switch (Constants.currentMode) {
        default -> new Gains(90.0);
        case SIM -> new Gains(75.0);
      };

  public record Gains(double kP) {}
}
