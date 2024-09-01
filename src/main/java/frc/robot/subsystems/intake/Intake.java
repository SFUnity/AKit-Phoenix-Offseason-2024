package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.GeneralUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.loggedShuffleboardClasses.LoggedShuffleboardBoolean;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Intake/Gains/kP", gains.kP());

  // In rotations
  private static final LoggedTunableNumber loweredAngle =
      new LoggedTunableNumber("Intake/Angles/lowered", 26);
  private static final LoggedTunableNumber raisedAngle =
      new LoggedTunableNumber("Intake/Angles/raised", 0);

  // In percent output
  private static final LoggedTunableNumber rollersSpeed =
      new LoggedTunableNumber("Intake/Speeds/intakeRollers", 1);
  private static final LoggedTunableNumber indexerSpeed =
      new LoggedTunableNumber("Intake/Speeds/indexer", 1);

  public final LoggedShuffleboardBoolean intakeWorking =
      new LoggedShuffleboardBoolean("Intake Working", "Intake", true)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(2, 1);

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

    // Update LEDs
    Leds.getInstance().intakeWorking = intakeWorking.get();

    // Logs
    measuredVisualizer.update(inputs.pivotPositionRots);
    setpointVisualizer.update(positionSetpoint);
    Logger.recordOutput("Intake/positionSetpointRotations", positionSetpoint);
    GeneralUtil.logSubsystem(this, "Intake");
  }

  private void indexerIn() {
    io.runIndexer(indexerSpeed.get());
  }

  private void indexerOut() {
    io.runIndexer(-indexerSpeed.get());
  }

  private void indexerStop() {
    io.runIndexer(0);
  }

  private void lower() {
    positionSetpoint = loweredAngle.get();
    io.setPivotPosition(positionSetpoint);
  }

  private void raise() {
    positionSetpoint = raisedAngle.get();
    io.setPivotPosition(positionSetpoint);
  }

  private void rollersIn() {
    io.runIntakeRollers(rollersSpeed.get());
  }

  private void rollersOut() {
    io.runIntakeRollers(-rollersSpeed.get());
  }

  private void rollersStop() {
    io.runIntakeRollers(0);
  }

  public Command raiseAndStopCmd() {
    return run(() -> {
          raise();
          rollersStop();
          indexerStop();
        })
        .withName("raise and stop");
  }

  public Command intakeCmd(Trigger lowerTrig) {
    return Commands.either(
        run(() -> {
              boolean lower = lowerTrig.getAsBoolean();
              indexerIn();
              if (lower) {
                lower();
                rollersIn();
              } else {
                raise();
                rollersStop();
              }
            })
            .withName("intake"),
        raiseAndStopCmd(),
        intakeWorking::get);
  }

  public Command poopCmd() {
    return Commands.either(
        run(() -> {
              raise();
              rollersOut();
              indexerOut();
            })
            .withName("poop"),
        raiseAndStopCmd(),
        intakeWorking::get);
  }
}
