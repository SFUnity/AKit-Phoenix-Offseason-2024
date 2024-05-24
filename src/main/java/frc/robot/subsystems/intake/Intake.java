package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import frc.robot.util.LoggedTunableNumber;

public class Intake {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/Gains/kP", gains.kP());

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
}
