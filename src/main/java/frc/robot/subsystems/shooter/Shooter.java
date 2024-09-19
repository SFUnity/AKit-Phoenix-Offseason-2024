package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.feeder.Feeder;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.pivot.Pivot;
import frc.robot.util.PoseManager;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.loggedShuffleboardClasses.LoggedShuffleboardBoolean;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends VirtualSubsystem {
  private final double kDistSensorRangeWhenNoteInches = 2.5;

  private final Flywheels flywheels;
  private final Feeder feeder;
  private final Pivot pivot;
  private final BeamBreakIO beamBreakIO;
  private final PoseManager poseManager;

  private final BeamBreakInputsAutoLogged beamBreakInputs = new BeamBreakInputsAutoLogged();
  private final LoggedShuffleboardBoolean beamBreakWorkingEntry =
      new LoggedShuffleboardBoolean("Beam Break Working", "Shooter", true);
  private boolean simNoteInShooter = true;

  public Shooter(
      Flywheels flywheels,
      Pivot pivot,
      BeamBreakIO beamBreakIO,
      Feeder feeder,
      PoseManager poseManager) {
    this.flywheels = flywheels;
    this.pivot = pivot;
    this.beamBreakIO = beamBreakIO;
    this.feeder = feeder;
    this.poseManager = poseManager;
  }

  public void periodic() {
    beamBreakIO.updateInputs(beamBreakInputs);
    Logger.processInputs("Shooter/BeamBreak", beamBreakInputs);
    Leds.getInstance().noteInShooter = noteInShooter();
  }

  /**
   * returns whether there is a note in the shooter
   *
   * @return boolean value of if there is a note in shooter
   */
  @AutoLogOutput
  public boolean noteInShooter() {
    if (Constants.currentMode != Constants.Mode.SIM) simNoteInShooter = true;
    return beamBreakInputs.distSensorRange <= kDistSensorRangeWhenNoteInches
        && distanceSensorWorking()
        && simNoteInShooter;
  }

  /**
   * returns whether the robot is on the opposing side of the field
   *
   * @return
   */
  public boolean onEnemyField() {
    // (TODO): should probably get a better way of obtaining this constant value
    return poseManager.getPose().getX() > 10.17059;
  }

  public boolean distanceSensorWorking() {
    return beamBreakInputs.isRangeValid && beamBreakWorkingEntry.get();
  }

  public void setSimNoteInShooter(boolean simNoteInShooter) {
    this.simNoteInShooter = simNoteInShooter;
  }

  public boolean atDesiredAngle() {
    return pivot.atDesiredAngle();
  }

  public Command setManualSpeakerShot() {
    return pivot
        .setManualSpeakerAngle()
        .alongWith(flywheels.shootSpeaker())
        .withName("setManualSpeakerShot");
  }

  public Command setAutoAimShot() {
    return pivot
        .setAutoSpeakerAngle()
        .alongWith(flywheels.shootSpeaker())
        .withName("setAutoAimShot");
  }

  public Command setAmpShot() {
    return pivot.setAmpAngle().alongWith(flywheels.shootAmp()).withName("setAmpShot");
  }

  public Command setFeeding() {
    return pivot.setFeedAngle().alongWith(flywheels.feed()).withName("setFeeding");
  }

  public Command setIntaking(LoggedShuffleboardBoolean intakeWorking) {
    return feeder
        .intake()
        .until(this::noteInShooter)
        .deadlineWith(pivot.setIntakeAngle().alongWith(flywheels.intake(intakeWorking)))
        .withName("setIntaking");
  }

  public Command setOuttaking() {
    return pivot
        .setIntakeAngle()
        .alongWith(Commands.waitUntil(pivot::atDesiredAngle).andThen(feeder.outtake()))
        .withName("setOuttaking");
  }

  public Command feedNoteToFlywheels() {
    return feeder.shoot().until(() -> !noteInShooter()).withName("feedNoteToFlywheels");
  }

  public Command stopFlywheels() {
    return flywheels.stop();
  }
}
