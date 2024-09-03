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

  public Command shoot() {
    // Note: something weird happens when you change to runEnd
    return Commands.startEnd(() -> io.runPercent(kShootingSpeedPercent), () -> io.runPercent(0));
  }

  public Command intake() {
    return Commands.startEnd(() -> io.runPercent(kIntakingSpeedPercent), () -> io.runPercent(0));
  }

  public Command outtake() {
    return Commands.startEnd(() -> io.runPercent(-kIntakingSpeedPercent), () -> io.runPercent(0));
  }
}
