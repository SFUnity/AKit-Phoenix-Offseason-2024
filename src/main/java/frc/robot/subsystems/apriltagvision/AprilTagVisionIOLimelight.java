package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Alert;
import frc.robot.util.LimelightHelpers9038;
import frc.robot.util.PoseManager;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {
  private String name;
  private final PoseManager poseManager;

  private final DoubleArraySubscriber observationSubscriber;

  private static final double disconnectedTimeout = 0.5;
  private final Alert disconnectedAlert;

  public AprilTagVisionIOLimelight(String camName, PoseManager poseManager) {
    name = camName;
    this.poseManager = poseManager;

    LimelightHelpers9038.setLEDMode_PipelineControl(name);
    LimelightHelpers9038.setCameraMode_Processor(name);

    var topic =
        LimelightHelpers9038.getLimelightNTTable("limelight")
            .getDoubleArrayTopic("botpose_orb_wpiblue");
    observationSubscriber =
        topic.subscribe(
            new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

    disconnectedAlert = new Alert("No data from: " + name, Alert.AlertType.ERROR);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    LimelightHelpers9038.SetRobotOrientation(
        "limelight", poseManager.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    TimestampedDoubleArray timestampedArray = observationSubscriber.getAtomic();
    double[] poseArray = timestampedArray.value;
    long observationTimestamp = timestampedArray.timestamp;

    inputs.estimatedPose = toPose2d(poseArray);
    inputs.isNew = observationTimestamp != 0;
    if (inputs.isNew) {
      inputs.timestamp = observationTimestamp / 1000000.0; // Convert from microseconds to seconds
    }
    inputs.tagCount = (int) poseArray[7];

    inputs.pipeline = LimelightHelpers9038.getCurrentPipelineIndex(name);
    inputs.ledMode = LimelightHelpers9038.getLimelightNTDouble(name, "ledMode");
    inputs.camMode = LimelightHelpers9038.getLimelightNTDouble(name, "camMode");

    // Update disconnected alert
    disconnectedAlert.set(Timer.getFPGATimestamp() - observationTimestamp < disconnectedTimeout);
  }

  @Override
  public void setPipeline(int pipelineIndex) {
    LimelightHelpers9038.setPipelineIndex(name, pipelineIndex);
  }

  private Pose2d toPose2d(double[] poseArray) {
    if (poseArray.length < 6) {
      System.err.println("Bad LL 2D Pose Data!");
      return new Pose2d();
    }
    Rotation2d rotation2d = new Rotation2d(Units.degreesToRadians(poseArray[5]));
    return new Pose2d(poseArray[0], poseArray[1], rotation2d);
  }
}
