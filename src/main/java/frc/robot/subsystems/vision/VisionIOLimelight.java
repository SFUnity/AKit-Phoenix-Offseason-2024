package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Alert;

public class VisionIOLimelight implements VisionIO {
  private static NetworkTable table;
  private static NetworkTableEntry tx, ty, tv, ta, tid, priorityid, pipeline, hb;
  private static NetworkTableEntry camMode;
  private static NetworkTableEntry ledMode;

  private static final double disconnectedTimeout = 0.5;
  private final Alert disconnectedAlert;
  private final Timer disconnectedTimer = new Timer();
  private double lastHB = 0;

  public VisionIOLimelight(String name) {
    table = NetworkTableInstance.getDefault().getTable(name);

    tx = table.getEntry("tx"); // Horizontal offset from crosshair to target (-29.8 to 29.8 degrees)
    ty = table.getEntry("ty"); // Vertical offset from crosshair to target (-24.85 to 24.85 degrees)
    tv = table.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1).
    ta = table.getEntry("ta"); // Target area (0% of image to 100% of image).
    tid = table.getEntry("tid"); // current id of the april tag

    priorityid = table.getEntry("priorityid"); // Preffered id of the april tag
    pipeline = table.getEntry("pipeline"); // Pipeline (0-9).
    ledMode = table.getEntry("ledMode"); // limelight's LED state (0-3).
    camMode = table.getEntry("camMode"); // limelight's operation mode (0-1).
    hb = table.getEntry("hb"); // heartbeat value. Increases once per frame, resets at 2 billion

    disconnectedAlert = new Alert("No data from: " + name, Alert.AlertType.ERROR);
    disconnectedTimer.start();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.targetXOffset = tx.getDouble(0) + 3; // 3 is a target offset we tuned
    inputs.targetYOffset = ty.getDouble(0);
    inputs.targetDetected = tv.getNumber(0).intValue() == 1;
    inputs.targetArea = ta.getDouble(0);
    inputs.targetID = tid.getDouble(0);

    inputs.priorityID = priorityid.getDouble(0);
    inputs.pipeline = pipeline.getDouble(0);
    inputs.ledMode = ledMode.getDouble(0);
    inputs.camMode = camMode.getDouble(0);

    // Update disconnected alert
    double currentHB = hb.getDouble(0);
    if (currentHB != lastHB) {
      lastHB = currentHB;
      disconnectedTimer.reset();
    }
    disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));
  }

  @Override
  public void setPipeline(int p) {
    pipeline.setDouble(p);
  }
}
