package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.pivot.Pivot;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.loggedShuffleboardClasses.LoggedShuffleboardBoolean;
import org.littletonrobotics.junction.Logger;

public class Shooter extends VirtualSubsystem {
  private final double kDistSensorRangeWhenNoteInches = 2.5;

  private final Flywheels flywheels;
  private final Pivot pivot;
  private final BeamBreakIO beamBreakIO;

  private final BeamBreakInputsAutoLogged beamBreakInputs = new BeamBreakInputsAutoLogged();
  private final LoggedShuffleboardBoolean beamBreakWorkingEntry =
      new LoggedShuffleboardBoolean("BeamBreak Working", "Shooter", true);

  public Shooter(Flywheels flywheels, Pivot pivot, BeamBreakIO beamBreakIO) {
    this.flywheels = flywheels;
    this.pivot = pivot;
    this.beamBreakIO = beamBreakIO;
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
    return pivot.setManualShootAngleCommand().alongWith(flywheels.shootSpeaker());
  }

  public Command setAutoAimShot() {
    return pivot.setAutoShootAngleCommand().alongWith(flywheels.shootSpeaker());
  }

  public Command setAmpShot() {
    return pivot.setAmpAngleCommand().alongWith(flywheels.shootAmp());
  }

  public Command setFeeding() {
    return pivot.setFeedAngleCommand().alongWith(flywheels.feed());
  }

  public Command setIntaking(boolean intakeWorking) {
    return flywheels.intake(intakeWorking); // add pivot command once written
  }
}
