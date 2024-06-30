package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Alert;
import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
  private String name;

  private static final double disconnectedTimeout = 0.5;
  private final Alert disconnectedAlert;
  private final Timer disconnectedTimer = new Timer();
  private double lastHB = 0;

  public VisionIOLimelight(String camName) {
    name = camName;

    LimelightHelpers.setLEDMode_PipelineControl(name);
    LimelightHelpers.setCameraMode_Processor(name);

    disconnectedAlert = new Alert("No data from: " + name, Alert.AlertType.ERROR);
    disconnectedTimer.start();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.targetXOffset = LimelightHelpers.getTX(name) + 3; // Horizontal offset from crosshair to target (-29.8 to 29.8 degrees). 3 is a target offset we tuned
    inputs.targetYOffset = LimelightHelpers.getTY(name); // Vertical offset from crosshair to target (-24.85 to 24.85 degrees)
    inputs.targetDetected = LimelightHelpers.getTV(name); // Whether the limelight has any valid targets (0 or 1).
    inputs.targetArea = LimelightHelpers.getTA(name); // Target area (0% of image to 100% of image).
    inputs.targetID = LimelightHelpers.getFiducialID(name); // current id of the april tag. Fiducial == AprilTag

    inputs.priorityID = LimelightHelpers.getLimelightNTDouble(name, "priorityid"); // Preffered id of the april tag
    inputs.pipeline = LimelightHelpers.getCurrentPipelineIndex(name);
    inputs.ledMode = LimelightHelpers.getLimelightNTDouble(name, "ledMode"); // 0 = pipeline control, 1 = force off, 2 = force blink, 3 = force on
    inputs.camMode = LimelightHelpers.getLimelightNTDouble(name, "camMode"); // 0 = processor, 1 = driver

    // Update disconnected alert
    double currentHB = LimelightHelpers.getLimelightNTDouble(name, "hb"); // heartbeat value. Increases once per frame, resets at 2 billion
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
