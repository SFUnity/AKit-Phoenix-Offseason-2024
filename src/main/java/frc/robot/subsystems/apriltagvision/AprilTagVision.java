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
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends VirtualSubsystem {
  private final AprilTagVisionIO io;
  private final AprilTagVisionIOInputsAutoLogged inputs = new AprilTagVisionIOInputsAutoLogged();
  private final PoseManager poseManager;

  private final LoggedTunableNumber stdDevMultiTagFactor =
      new LoggedTunableNumber("Vision/stdDevMultiTagFactor", 0.2);
  private final LoggedTunableNumber stdDevSlopeDistance =
      new LoggedTunableNumber("Vision/stdDevSlopeDistance", 0.10);
  private final LoggedTunableNumber stdDevPowerDistance =
      new LoggedTunableNumber("Vision/stdDevPowerDistance", 2.0);
  private final LoggedTunableNumber minTagArea = new LoggedTunableNumber("Vision/minTagAreaPercentOfImage", 3.0);

  public AprilTagVision(AprilTagVisionIO io, PoseManager poseManager) {
    this.io = io;
    this.poseManager = poseManager;

    io.setPipeline(Pipelines.BLUE_SPEAKER);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AprilTagVision", inputs);

    Leds.getInstance().tagsDetected = inputs.tagCount > 0;

    Pose2d estimatedPose = inputs.estimatedPose;
    // Exit if there are no tags in sight or the pose is blank
    if (inputs.tagCount == 0 || estimatedPose.equals(new Pose2d())) return;

    // Exit if the estimated pose is off the field
    if (estimatedPose.getX() < -fieldBorderMargin
        || estimatedPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
        || estimatedPose.getY() < -fieldBorderMargin
        || estimatedPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) return;

    // Exit if the estimated pose is too far away from current pose
    double allowableDistance = inputs.tagCount; // In meters
    if (poseManager.getDistanceTo(estimatedPose) > allowableDistance) return;

    // Exit if the tags are too small
    if (inputs.avgTagArea < minTagArea.get()) return;

    // Create stdDevs
    Matrix<N3, N1> stdDevs = VecBuilder.fill(.7, .7, 100);
    if (inputs.tagCount == 1) {
      // Change std devs based on distance to the tag
      stdDevs =
          stdDevs.times(
              stdDevSlopeDistance.get() * (Math.pow(inputs.avgTagDist, stdDevPowerDistance.get())));
    } else {
      // Decrease std devs if multiple targets are visible
      stdDevs = stdDevs.times(stdDevMultiTagFactor.get());
    }

    // Add result because all checks passed
    poseManager.addVisionMeasurement(estimatedPose, inputs.timestamp, stdDevs, inputs.tagCount);
  }
}
