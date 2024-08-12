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
import frc.robot.util.PoseManager;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends VirtualSubsystem {
  private final AprilTagVisionIO io;
  private final AprilTagVisionIOInputsAutoLogged inputs = new AprilTagVisionIOInputsAutoLogged();
  private final PoseManager poseManager;

  public AprilTagVision(AprilTagVisionIO io, PoseManager poseManager) {
    this.io = io;
    this.poseManager = poseManager;

    io.setPipeline(Pipelines.BLUE_SPEAKER);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AprilTagVision", inputs);

    Leds.getInstance().tagsDetected = inputs.tagCount > 0;

    Pose2d robotPose = inputs.estimatedPose;
    // Exit if there are no tags in sight or the pose is blank
    if (inputs.tagCount == 0 || robotPose.equals(new Pose2d())) {
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
    Matrix<N3, N1> stdDevs = VecBuilder.fill(.7, .7, 9999999);
    poseManager.addVisionMeasurement(robotPose, inputs.timestamp, stdDevs, inputs.tagCount);
  }
}
