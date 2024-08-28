package frc.robot.subsystems.shooter;

import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.pivot.Pivot;

public class Shooter {
  private final Flywheel flywheel;
  private final Pivot pivot;

  public Shooter(Flywheel flywheel, Pivot pivot) {
    this.flywheel = flywheel;
    this.pivot = pivot;
  }
}
