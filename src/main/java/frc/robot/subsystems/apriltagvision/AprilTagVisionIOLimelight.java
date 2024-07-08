package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Alert;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.PoseManager;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {
  private String name;
  private final PoseManager poseManager;

  private static final double disconnectedTimeout = 0.5;
  private final Alert disconnectedAlert;
  private final Timer disconnectedTimer = new Timer();

  public AprilTagVisionIOLimelight(String camName, PoseManager poseManager) {
    name = camName;
    this.poseManager = poseManager;

    LimelightHelpers.setLEDMode_PipelineControl(name);
    LimelightHelpers.setCameraMode_Processor(name);

    disconnectedAlert = new Alert("No data from: " + name, Alert.AlertType.ERROR);
    disconnectedTimer.start();
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    LimelightHelpers.SetRobotOrientation(
        "limelight", poseManager.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    inputs.estimatedPose = mt2.pose;
    inputs.timestamp = mt2.timestampSeconds; // take cares of latency for you
    inputs.tagCount = mt2.tagCount;

    inputs.pipeline = LimelightHelpers.getCurrentPipelineIndex(name);
    inputs.ledMode = LimelightHelpers.getLimelightNTDouble(name, "ledMode");
    inputs.camMode = LimelightHelpers.getLimelightNTDouble(name, "camMode");
    inputs.hb = LimelightHelpers.getLimelightNTDouble(name, "hb");

    // Update disconnected alert
    var queue = LimelightHelpers.getLimelightNTTableEntry(name, "botpose_orb_wpiblue").readQueue();
    inputs.isNew = queue.length > 0;
    if (inputs.isNew) {
      disconnectedTimer.reset();
    }
    disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));
  }

  @Override
  public void setPipeline(int p) {
    LimelightHelpers.setPipelineIndex(name, p);
  }
}
