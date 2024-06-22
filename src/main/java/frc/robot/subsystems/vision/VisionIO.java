package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public double targetXOffset = 0;
    public double targetYOffset = 0;
    public boolean targetDetected = false;
    public double targetArea = 0;
    public double targetID = 0; // For aprilTags

    public double priorityID = 0; // For aprilTags
    public double pipeline = 0; // TODO store the pipelines where the code can see them
    public double ledMode = 0;
    public double camMode = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Sets the pipeline number. */
  public default void setPipeline(int pipeline) {}
}
