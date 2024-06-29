package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class GeneralUtil {
  public static void logSubsystem(SubsystemBase s, String sName) {
    sName += "/cmdInfo/";
    Logger.recordOutput(sName + "hasDefault", s.getDefaultCommand() != null);
    Logger.recordOutput(
        sName + "default",
        s.getDefaultCommand() != null ? s.getDefaultCommand().getName() : "none");
    Logger.recordOutput(sName + "hasCommand", s.getCurrentCommand() != null);
    Logger.recordOutput(
        sName + "command",
        s.getCurrentCommand() != null ? s.getCurrentCommand().getName() : "none");
  }
}
