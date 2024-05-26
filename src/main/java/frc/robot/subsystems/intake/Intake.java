package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Intake/Gains/kP", gains.kP());

  private static final LoggedTunableNumber loweredAngle =
      new LoggedTunableNumber("Intake/Angles/lowered", kLoweredAngle);
  private static final LoggedTunableNumber raisedAngle =
      new LoggedTunableNumber("Intake/Angles/raised", kRaisedAngle);
  private static final LoggedTunableNumber climbingAngle =
      new LoggedTunableNumber("Intake/Angles/climbing", kClimbingAngle);

  private static final LoggedTunableNumber rollersSpeed =
      new LoggedTunableNumber("Intake/Speeds/intakeRollers", kRollersSpeed);
  private static final LoggedTunableNumber indexerSpeed =
      new LoggedTunableNumber("Intake/Speeds/indexer", kIndexerSpeed);

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;

    io.setPID(kP.get());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Update controllers
    LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get()), kP);
  }

  public void indexerIn() {
    io.runIndexer(indexerSpeed.get());
  }

  public void indexerOut() {
    io.runIndexer(-indexerSpeed.get());
  }

  public void indexerStop() {
    io.runIndexer(0);
  }

  public void lower() {
    io.setPivotPosition(loweredAngle.get());
  }

  public void raise() {
    io.setPivotPosition(raisedAngle.get());
  }

  public void climb() {
    io.setPivotPosition(climbingAngle.get());
  }

  public void in() {
    io.runIntakeRollers(rollersSpeed.get());
  }

  public void out() {
    io.runIntakeRollers(-rollersSpeed.get());
  }

  public void stopRollers() {
    io.runIntakeRollers(0);
  }

  public void stopAll() {
    io.stop();
  }
}
