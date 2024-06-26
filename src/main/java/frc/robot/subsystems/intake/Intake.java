package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Intake/Gains/kP", gains.kP());

  // In rotations
  private static final LoggedTunableNumber loweredAngle =
      new LoggedTunableNumber("Intake/Angles/lowered", 26);
  private static final LoggedTunableNumber raisedAngle =
      new LoggedTunableNumber("Intake/Angles/raised", 0);
  private static final LoggedTunableNumber climbingAngle =
      new LoggedTunableNumber("Intake/Angles/climbing", 5);

  // In percent output
  private static final LoggedTunableNumber rollersSpeed =
      new LoggedTunableNumber("Intake/Speeds/intakeRollers", 1);
  private static final LoggedTunableNumber indexerSpeed =
      new LoggedTunableNumber("Intake/Speeds/indexer", 1);

  private double positionSetpoint = 0;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final IntakeVisualizer measuredVisualizer;
  private final IntakeVisualizer setpointVisualizer;

  public Intake(IntakeIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        io.setPID(0.08);
        break;
      case SIM:
        io.setPID(20);
        break;
      default:
        break;
    }

    measuredVisualizer = new IntakeVisualizer("Measured", Color.kRed);
    setpointVisualizer = new IntakeVisualizer("Setpoint", Color.kBlue);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Update controllers
    LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get()), kP);

    // Logs
    measuredVisualizer.update(inputs.pivotPositionRots);
    setpointVisualizer.update(positionSetpoint);
    Logger.recordOutput("Intake/positionSetpointRotations", positionSetpoint);
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
    positionSetpoint = loweredAngle.get();
    io.setPivotPosition(positionSetpoint);
  }

  public void raise() {
    positionSetpoint = raisedAngle.get();
    io.setPivotPosition(positionSetpoint);
  }

  public void climb() {
    io.setPivotPosition(climbingAngle.get());
  }

  public void rollersIn() {
    io.runIntakeRollers(rollersSpeed.get());
  }

  public void rollersOut() {
    io.runIntakeRollers(-rollersSpeed.get());
  }

  public void rollersStop() {
    io.runIntakeRollers(0);
  }

  public void stopAll() {
    io.stop();
  }

  public Command lowerAndRunIntakeCmd() {
        return run(() -> {
            lower();
            rollersIn();
            indexerIn();
        });
    }

    public Command raiseAndStopIntakeCmd() {
        return runOnce(() -> {
            raise();
            rollersStop();
            indexerStop();
        });
    }
}
