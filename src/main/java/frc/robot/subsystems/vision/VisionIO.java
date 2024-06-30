package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    /** Horizontal offset from crosshair to target (-29.8 to 29.8 degrees). 3 is a target offset we tuned */
    public double targetXOffset = 0;
    /** Vertical offset from crosshair to target (-24.85 to 24.85 degrees) */
    public double targetYOffset = 0;
    /** Whether the limelight has any valid targets */
    public boolean targetDetected = false;
    /** Target area (0% of image to 100% of image) */
    public double targetArea = 0;
    /** ID of the primary in-view AprilTag */
    public double targetID = 0;

    public double pipeline = 0; // TODO store the pipelines where the code can see them
    /** 0 = pipeline control, 1 = force off, 2 = force blink, 3 = force on */
    public double ledMode = 0;
    /** 0 = processor, 1 = driver */
    public double camMode = 0;
    /** heartbeat value. Increases once per frame, resets at 2 billion */
    public double hb = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Sets the pipeline number. */
  public default void setPipeline(int pipeline) {}
}
