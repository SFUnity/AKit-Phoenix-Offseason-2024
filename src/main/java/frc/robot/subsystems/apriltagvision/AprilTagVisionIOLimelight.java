package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Alert;
import frc.robot.util.LimelightHelpers9038;
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

    LimelightHelpers9038.setLEDMode_PipelineControl(name);
    LimelightHelpers9038.setCameraMode_Processor(name);

    disconnectedAlert = new Alert("No data from: " + name, Alert.AlertType.ERROR);
    disconnectedTimer.start();
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    LimelightHelpers9038.SetRobotOrientation(
        "limelight", poseManager.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers9038.PoseEstimate mt2 = LimelightHelpers9038.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    inputs.estimatedPose = mt2.pose;
    inputs.timestamp = mt2.timestampSeconds; // take cares of latency for you
    inputs.tagCount = mt2.tagCount;

    inputs.pipeline = LimelightHelpers9038.getCurrentPipelineIndex(name);
    inputs.ledMode = LimelightHelpers9038.getLimelightNTDouble(name, "ledMode");
    inputs.camMode = LimelightHelpers9038.getLimelightNTDouble(name, "camMode");
    inputs.hb = LimelightHelpers9038.getLimelightNTDouble(name, "hb");

    // Update disconnected alert
    var queue = LimelightHelpers9038.getLimelightNTTableEntry(name, "botpose_orb_wpiblue").readQueue();
    inputs.isNew = queue.length > 0;
    if (inputs.isNew) {
      disconnectedTimer.reset();
    }
    disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));
  }

  @Override
  public void setPipeline(int p) {
    LimelightHelpers9038.setPipelineIndex(name, p);
  }
}
