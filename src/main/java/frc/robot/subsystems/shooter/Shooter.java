package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.pivot.Pivot;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Shooter {
  private final Flywheel flywheel;
  private final Pivot pivot;

  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  public Shooter(Flywheel flywheel, Pivot pivot) {
    this.flywheel = flywheel;
    this.pivot = pivot;
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
