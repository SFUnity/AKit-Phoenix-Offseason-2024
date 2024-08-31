package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.feeder.Feeder;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.pivot.Pivot;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.loggedShuffleboardClasses.LoggedShuffleboardBoolean;
import org.littletonrobotics.junction.Logger;

public class Shooter extends VirtualSubsystem {
  private final double kDistSensorRangeWhenNoteInches = 2.5;

  private final Flywheels flywheels;
  private final Feeder feeder;
  private final Pivot pivot;
  private final BeamBreakIO beamBreakIO;

  private final BeamBreakInputsAutoLogged beamBreakInputs = new BeamBreakInputsAutoLogged();
  private final LoggedShuffleboardBoolean beamBreakWorkingEntry =
      new LoggedShuffleboardBoolean("BeamBreak Working", "Shooter", true);

  public Shooter(Flywheels flywheels, Pivot pivot, BeamBreakIO beamBreakIO, Feeder feeder) {
    this.flywheels = flywheels;
    this.pivot = pivot;
    this.beamBreakIO = beamBreakIO;
    this.feeder = feeder;
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
  // May make sense to change this to return a boolean. TBD
  public boolean noteInShooter() {
    return beamBreakInputs.distSensorRange <= kDistSensorRangeWhenNoteInches
        && distanceSensorWorking();
  }

  public boolean distanceSensorWorking() {
    return beamBreakInputs.isRangeValid && beamBreakWorkingEntry.get();
  }

  public Command setManualSpeakerShot() {
    return pivot
        .setManualShootAngleCommand()
        .alongWith(flywheels.shootSpeaker())
        .withName("setManualSpeakerShot");
  }

  public Command setAutoAimShot() {
    return pivot
        .setAutoShootAngleCommand()
        .alongWith(flywheels.shootSpeaker())
        .withName("setManualSpeakerShot");
  }

  public Command setAmpShot() {
    return pivot
        .setAmpAngleCommand()
        .alongWith(flywheels.shootAmp())
        .withName("setAmpShot");
  }

  public Command setFeeding() {
    return pivot
        .setFeedAngleCommand()
        .alongWith(flywheels.feed())
        .withName("setFeeding");
  }

  public Command setIntaking(LoggedShuffleboardBoolean intakeWorking) {
    return pivot
        .setIntakeAngleCommand()
        .alongWith(flywheels.intake(intakeWorking))
        .alongWith(feeder.feederIntake().until(this::noteInShooter))
        .withName("setIntaking");
  }

  public Command setOuttaking() {
    return feeder.feederOuttake().withName("setOuttaking"); // add pivot command once written
  }

  public Command feedNoteToFlywheels() {
    return feeder.feederShoot().until(() -> !noteInShooter()).withName("feedNoteToFlywheels");
  }

  public Command stopFlywheels() {
    return flywheels.stop();
  }
}
