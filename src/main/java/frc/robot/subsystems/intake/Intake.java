package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import frc.robot.util.LoggedTunableNumber;

public class Intake {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Gains/kP", gains.kP());
}
