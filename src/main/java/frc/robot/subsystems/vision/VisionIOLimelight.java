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
    inputs.targetXOffset = LimelightHelpers.getTX(name) + 3; // 3 is a target offset we tuned
    inputs.targetYOffset = LimelightHelpers.getTY(name);
    inputs.targetDetected = LimelightHelpers.getTV(name);
    inputs.targetArea = LimelightHelpers.getTA(name);
    inputs.targetID = LimelightHelpers.getFiducialID(name); // Fiducial == AprilTag

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
