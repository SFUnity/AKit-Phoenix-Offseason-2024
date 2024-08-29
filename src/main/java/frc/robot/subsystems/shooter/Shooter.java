package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.pivot.Pivot;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.loggedShuffleboardClasses.LoggedShuffleboardBoolean;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Shooter extends VirtualSubsystem{
  private final double kDistSensorRangeWhenNoteInches = 2.5;

  private final Flywheel flywheel;
  private final Pivot pivot;
  private final BeamBreakIO beamBreakIO;

  private final BeamBreakInputsAutoLogged beamBreakInputs = new BeamBreakInputsAutoLogged();
  private final LoggedShuffleboardBoolean beamBreakWorkingEntry =
      new LoggedShuffleboardBoolean("BeamBreak Working", "Shooter", true);

  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  public Shooter(Flywheel flywheel, Pivot pivot, BeamBreakIO beamBreakIO) {
    this.flywheel = flywheel;
    this.pivot = pivot;
    this.beamBreakIO = beamBreakIO;
  }

  public void periodic() {
    beamBreakIO.updateInputs(beamBreakInputs);
    Logger.processInputs("BeamBreak", beamBreakInputs);
  }

  /**
   * returns whether there is a note in the shooter
   *
   * @return boolean value of if there is a note in shooter
   */
  // May make sense to change this to return a boolean. TBD
  public boolean noteInShooter() {
    return beamBreakInputs.distSensorRange <= kDistSensorRangeWhenNoteInches && distanceSensorWorking();
  }

  public boolean distanceSensorWorking() {
    return beamBreakInputs.isRangeValid && beamBreakWorkingEntry.get();
  }

  public Command runFlywheelCmd() {
    return Commands.startEnd(
            () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
        .withName("Run Flywheel");
  }

  public Command flywheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return flywheel.sysIdQuasistatic(direction);
  }

  public Command flywheelSysIdDynamic(SysIdRoutine.Direction direction) {
    return flywheel.sysIdDynamic(direction);
  }

  public Command setManualShootAngleCommand() {
    return pivot.setManualShootAngleCommand();
  }
}
