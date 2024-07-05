package frc.robot.subsystems.apriltagvision;

public class AprilTagVisionConstants {
  public static final double fieldBorderMargin = 0.5;

  /** how many degrees back is your limelight rotated from perfectly vertical? */
  public static final double limelightMountAngleDegrees = 28.0;
  /** distance from the center of the Limelight lens to the floor */
  public static final double limelightLensHeightInches = 26.5;

  /** distance from the speaker tag to the floor */
  public static final double heightOfTagInches = 56.5;
  /** height of speaker */
  public static final double heightOfSpeakerInches = 80;

  public static enum Pipelines {
    // blue speaker = 0, red speaker = 1, source = 2
    BLUE_SPEAKER,
    RED_SPEAKER,
    SOURCE;

    public int get() {
      return ordinal();
    }
  }
}
