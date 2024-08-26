package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.*;
import frc.robot.util.Alert;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.PoseManager;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {
  private String name;
  private final PoseManager poseManager;

  private static final double disconnectedTimeout = 0.5;
  private final Alert disconnectedAlert;
  private double lastTimestamp = 0;

  private final double DEFAUlT_CROP = 0.9;
  private final double CROP_BUFFER = 0.1;

  public AprilTagVisionIOLimelight(String camName, PoseManager poseManager) {
    name = camName;
    this.poseManager = poseManager;

    LimelightHelpers.setLEDMode_PipelineControl(name);

    disconnectedAlert = new Alert("No data from: " + name, Alert.AlertType.ERROR);

    resetCropping();
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    LimelightHelpers.SetRobotOrientation(
        "limelight", poseManager.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate observation =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    inputs.estimatedPose = observation.pose;
    inputs.timestamp = observation.timestampSeconds;
    inputs.tagCount = observation.tagCount;
    inputs.avgTagDist = observation.avgTagDist;

    inputs.pipeline = LimelightHelpers.getCurrentPipelineIndex(name);
    inputs.ledMode = LimelightHelpers.getLimelightNTDouble(name, "ledMode");

    // Update disconnected alert
    if (observation.timestampSeconds != 0) {
      lastTimestamp = observation.timestampSeconds;
    }
    disconnectedAlert.set(Timer.getFPGATimestamp() - lastTimestamp < disconnectedTimeout);

    dynamicCropping();
  }

  @Override
  public void setPipeline(int pipelineIndex) {
    LimelightHelpers.setPipelineIndex(name, pipelineIndex);
  }

  @Override
  public void setPipeline(Pipelines pipelineEnum) {
    LimelightHelpers.setPipelineIndex(name, Pipelines.getIndexFor(pipelineEnum));
  }

  private void dynamicCropping() {
    double[] tcornxy = LimelightHelpers.getLimelightNTDoubleArray(name, "tcornxy");
    if (tcornxy.length == 0) {
      resetCropping();
      return;
    }

    double minX = 0;
    double maxX = 0;
    double minY = 0;
    double maxY = 0;

    // Iterate over all tag corners
    for (int i = 0; i < tcornxy.length / 2; i += 2) {
      minX = Math.min(minX, tcornxy[i]);
      maxX = Math.max(maxX, tcornxy[i]);
    }
    for (int i = 1; i < tcornxy.length / 2; i += 2) {
      minY = Math.min(minY, tcornxy[i]);
      maxY = Math.max(maxY, tcornxy[i]);
    }

    // Apply crop buffer and clamp to default crop size
    double cropXMin = MathUtil.clamp(minX - CROP_BUFFER, -DEFAUlT_CROP, DEFAUlT_CROP);
    double cropXMax = MathUtil.clamp(maxX + CROP_BUFFER, -DEFAUlT_CROP, DEFAUlT_CROP);
    double cropYMin = MathUtil.clamp(minY - CROP_BUFFER, -DEFAUlT_CROP, DEFAUlT_CROP);
    double cropYMax = MathUtil.clamp(maxY + CROP_BUFFER, -DEFAUlT_CROP, DEFAUlT_CROP);

    LimelightHelpers.setCropWindow(name, cropXMin, cropXMax, cropYMin, cropYMax);
  }

  private void resetCropping() {
    LimelightHelpers.setCropWindow(name, -DEFAUlT_CROP, DEFAUlT_CROP, -DEFAUlT_CROP, DEFAUlT_CROP);
  }
}
