package frc.robot.subsystems.apriltagvision;

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.FieldConstants;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.Pipelines;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends VirtualSubsystem {
  private final AprilTagVisionIO io;
  private final AprilTagVisionIOInputsAutoLogged inputs = new AprilTagVisionIOInputsAutoLogged();

  private VisionResult result = null;

  public AprilTagVision(AprilTagVisionIO io) {
    this.io = io;

    io.setPipeline(Pipelines.BLUE_SPEAKER.get());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    Leds.getInstance().tagsDetected = inputs.tagCount > 0;
    // TODO figure out new way to align with speaker

    // Clear previous result
    result = null;

    Pose2d robotPose = inputs.estimatedPose;
    boolean doRejectUpdate = false;

    while (!doRejectUpdate) {
      // TODO refactor to use if statements
      // Exit if old data
      doRejectUpdate = !inputs.isNew;
      // Exit if there are no tags
      doRejectUpdate = inputs.tagCount == 0;
      // Exit if robot pose is off the field
      doRejectUpdate =
          robotPose.getX() < -fieldBorderMargin
              || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
              || robotPose.getY() < -fieldBorderMargin
              || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin;
      // Add result because all checks passed
      result = new VisionResult(robotPose, inputs.timestamp, VecBuilder.fill(.7, .7, 9999999));
    }
  }

  /** Returns the last recorded pose */
  public VisionResult getResult() {
    return result;
  }

  public double getTagCount() {
    return inputs.tagCount;
  }

  public record VisionResult(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
