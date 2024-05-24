package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import frc.robot.util.LoggedTunableNumber;

public class Intake {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/Gains/kP", gains.kP());

  private static final LoggedTunableNumber loweredAngle = new LoggedTunableNumber("Intake/Angles/lowered", kLoweredAngle);
  private static final LoggedTunableNumber raisedAngle = new LoggedTunableNumber("Intake/Angles/raised", kRaisedAngle);
  private static final LoggedTunableNumber climbingAngle = new LoggedTunableNumber("Intake/Angles/climbing", kClimbingAngle);

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
}
