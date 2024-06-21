package frc.robot.subsystems.vision;

public class Vision {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public Vision(VisionIO io) {
    this.io = io;
  }
}
