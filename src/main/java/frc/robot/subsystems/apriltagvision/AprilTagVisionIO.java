package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagVisionIO {
  @AutoLog
  public static class AprilTagVisionIOInputs {
    public Pose2d estimatedPose;
    public double timestamp;
    public boolean isNewPose;
    public int tagCount;

    public double pipeline = 0; // TODO store the pipelines where the code can see them
    /** 0 = pipeline control, 1 = force off, 2 = force blink, 3 = force on */
    public double ledMode = 0;
    /** 0 = processor, 1 = driver */
    public double camMode = 0;
    /** heartbeat value. Increases once per frame, resets at 2 billion */
    public double hb = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AprilTagVisionIOInputs inputs) {}

  /** Sets the pipeline number. */
  public default void setPipeline(int pipeline) {}
}
