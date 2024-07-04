package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Alert;
import frc.robot.util.LimelightHelpers;
import java.util.function.DoubleSupplier;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {
  private String name;

  private static final double disconnectedTimeout = 0.5;
  private final Alert disconnectedAlert;
  private final Timer disconnectedTimer = new Timer();
  private double lastHB = 0;

  private Pose2d lastEstimatedPose = null;
  private DoubleSupplier robotYawInDegrees;

  public AprilTagVisionIOLimelight(String camName, DoubleSupplier robotYawInDegrees) {
    name = camName;
    this.robotYawInDegrees = robotYawInDegrees;

    LimelightHelpers.setLEDMode_PipelineControl(name);
    LimelightHelpers.setCameraMode_Processor(name);

    disconnectedAlert = new Alert("No data from: " + name, Alert.AlertType.ERROR);
    disconnectedTimer.start();
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    LimelightHelpers.SetRobotOrientation(
        "limelight", robotYawInDegrees.getAsDouble(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    inputs.estimatedPose = mt2.pose;
    inputs.timestamp = mt2.timestampSeconds; // take cares of latency for you
    inputs.tagCount = mt2.tagCount;
    inputs.isNewPose = inputs.estimatedPose != lastEstimatedPose;
    lastEstimatedPose = inputs.estimatedPose;

    inputs.pipeline = LimelightHelpers.getCurrentPipelineIndex(name);
    inputs.ledMode = LimelightHelpers.getLimelightNTDouble(name, "ledMode");
    inputs.camMode = LimelightHelpers.getLimelightNTDouble(name, "camMode");
    inputs.hb = LimelightHelpers.getLimelightNTDouble(name, "hb");

    // Update disconnected alert
    double currentHB = inputs.hb;
    if (currentHB != lastHB) {
      lastHB = currentHB;
      disconnectedTimer.reset();
    }
    disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));
  }

  @Override
  public void setPipeline(int p) {
    LimelightHelpers.setPipelineIndex(name, p);
  }
}
