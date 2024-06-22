package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public Vision(VisionIO io) {
    this.io = io;

    io.setPipeline(Pipelines.BLUE_SPEAKER.get());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
  }

  public boolean alignedWithTag() {
    if (inputs.targetDetected) {
      System.out.println("No tag in sight"); // TODO make an Alert
    }
    return Math.abs(inputs.targetXOffset) < 2;
  }

  public double getDistance () {
    double angleToGoalDegrees = limelightMountAngleDegrees + inputs.targetYOffset;
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

    // calculate distance
    return (heightOfTagInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  }
}
