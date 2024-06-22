package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelight implements VisionIO {
  private static NetworkTable table;
  private static NetworkTableEntry tx, ty, tv, ta, tid, priorityid, pipeline;
  private static NetworkTableEntry camMode;
  private static NetworkTableEntry ledMode;

  public VisionIOLimelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    tx =
        table.getEntry("tx"); // Horizontal offset from crosshair to target (-29.8 to 29.8 degrees).
    ty =
        table.getEntry("ty"); // Vertical offset from crosshair to target (-24.85 to 24.85 degrees).
    tv = table.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1).
    ta = table.getEntry("ta"); // Target area (0% of image to 100% of image).
    tid = table.getEntry("tid"); // current id of the april tag

    priorityid = table.getEntry("priorityid"); // Preffered id of the april tag
    pipeline = table.getEntry("pipeline"); // Pipeline (0-9).
    ledMode = table.getEntry("ledMode"); // limelight's LED state (0-3).
    camMode = table.getEntry("camMode"); // limelight's operation mode (0-1).
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.targetXOffset = tx.getDouble(0) + 3;
    inputs.targetYOffset = ty.getDouble(0);
    inputs.targetDetected = tv.getNumber(0).intValue() == 1;
    inputs.targetArea = ta.getDouble(0);
    inputs.targetID = tid.getDouble(0);

    inputs.priorityID = priorityid.getDouble(0);
    inputs.pipeline = pipeline.getDouble(0);
    inputs.ledMode = ledMode.getDouble(0);
    inputs.camMode = camMode.getDouble(0);
  }

  @Override
  public void setPipeline(int p) {
    pipeline.setDouble(p);
  }
}
