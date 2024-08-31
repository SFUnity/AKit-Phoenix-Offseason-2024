package frc.robot.subsystems.shooter.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private final double kShootingSpeedPercent = 1.0;
  private final double kIntakingSpeedPercent = 0.13;

  public Feeder(FeederIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Feeder", inputs);

    // Logs
    GeneralUtil.logSubsystem(this, "Shooter/Feeder");
  }

  private void shoot() {
    io.runPercent(kShootingSpeedPercent);
  }

  private void intake() {
    io.runPercent(kIntakingSpeedPercent);
  }

  public Command feederShoot() {
    return run(this::shoot);
  }

  public Command feederIntake() {
    return Commands.startEnd(this::intake, () -> io.runPercent(0));
  }
}
