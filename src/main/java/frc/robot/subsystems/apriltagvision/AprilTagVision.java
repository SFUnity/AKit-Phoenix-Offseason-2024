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
    Logger.processInputs("AprilTagVision", inputs);

    Leds.getInstance().tagsDetected = inputs.tagCount > 0;
    // TODO figure out new way to align with speaker

    // Clear previous result
    result = null;

    Pose2d robotPose = inputs.estimatedPose;
    // Exit if data is old or there are no tags in sight
    if (!inputs.isNew || inputs.tagCount == 0) {
      return;
    }
    // Exit if the estimated pose is off the field
    if (robotPose.getX() < -fieldBorderMargin
        || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
        || robotPose.getY() < -fieldBorderMargin
        || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) {
      return;
    }
    // Add result because all checks passed
    result = new VisionResult(robotPose, inputs.timestamp, VecBuilder.fill(.7, .7, 9999999));
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
