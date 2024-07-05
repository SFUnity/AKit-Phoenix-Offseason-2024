package frc.robot.subsystems.apriltagvision;

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.Pipelines;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends VirtualSubsystem {
  private final AprilTagVisionIO io;
  private final AprilTagVisionIOInputsAutoLogged inputs = new AprilTagVisionIOInputsAutoLogged();

  public AprilTagVision(AprilTagVisionIO io) {
    this.io = io;

    io.setPipeline(Pipelines.BLUE_SPEAKER.get());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    Leds.getInstance().tagsDetected = inputs.tagCount > 0;
    // TODO figure out new way to align with speaker

    Pose2d robotPose = inputs.estimatedPose;
    // Exit if robot pose is off the field
    if (robotPose.getX() < -fieldBorderMargin
        || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
        || robotPose.getY() < -fieldBorderMargin
        || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) {
      // Ignore tag
    }
  }

  public double getDistance() {
    // ! now returns a fake value
    double angleToGoalDegrees = limelightMountAngleDegrees; // + inputs.targetYOffset;
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
    // TODO find out new way to do this using pose
    // calculate distance
    return (heightOfTagInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  }
}
