package frc.robot.subsystems.apriltagvision;

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.Pipelines;
import frc.robot.subsystems.leds.Leds;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
  private final AprilTagVisionIO io;
  private final AprilTagVisionIOInputsAutoLogged inputs = new AprilTagVisionIOInputsAutoLogged();

  public AprilTagVision(AprilTagVisionIO io) {
    this.io = io;

    io.setPipeline(Pipelines.BLUE_SPEAKER.get());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    Leds.getInstance().targetDetected = inputs.targetDetected;
    Leds.getInstance().alignedWithTarget = alignedWithTag();
  }

  public boolean alignedWithTag() {
    return Math.abs(inputs.targetXOffset) < 2;
  }

  public double getDistance() {
    double angleToGoalDegrees = limelightMountAngleDegrees + inputs.targetYOffset;
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

    // calculate distance
    return (heightOfTagInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  }
}
