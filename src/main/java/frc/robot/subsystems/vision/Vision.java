package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConstants.Pipelines;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  private final Alert noTagDetectedAlert = new Alert("No tag detected", AlertType.WARNING);

  public Vision(VisionIO io) {
    this.io = io;

    io.setPipeline(Pipelines.BLUE_SPEAKER.get());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    noTagDetectedAlert.set(!inputs.targetDetected); // TODO remove once LEDs are added
  }

  public boolean alignedWithTag() {
    return Math.abs(inputs.targetXOffset) < 2;
  }

  public double getDistance () {
    double angleToGoalDegrees = limelightMountAngleDegrees + inputs.targetYOffset;
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

    // calculate distance
    return (heightOfTagInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  }
}
