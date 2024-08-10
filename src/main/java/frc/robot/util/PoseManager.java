package frc.robot.util;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class PoseManager {
  private SwerveModulePosition[] lastModulePositions = // For reseting pose
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d lastGyroAngle = new Rotation2d();
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kinematics, lastGyroAngle, lastModulePositions, new Pose2d());
  private double lastYawVelocity = 0.0;

  public PoseManager() {}

  public void addOdometryMeasurement(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, double yawVelocity) {
    lastModulePositions = modulePositions;
    lastGyroAngle = gyroAngle;
    poseEstimator.update(gyroAngle, modulePositions);
    lastYawVelocity = yawVelocity;
  }

  public void addVisionMeasurement(AprilTagVision.VisionResult visionResult, int tagCount) {
    // Exit if the robot's angular velocity is too high
    if (Math.abs(lastYawVelocity) > 720) {
      return;
    }
    // Exit if the estimated pose is too far away from current pose
    double allowableDistance = tagCount * 3; // In meters
    Pose2d resultPose = visionResult.pose();
    if (getDistanceTo(resultPose) > allowableDistance) {
      return;
    }
    // Add result because all checks passed
    poseEstimator.addVisionMeasurement(
        visionResult.pose(), visionResult.timestamp(), visionResult.stdDevs());
  }

  public double getDistanceTo(Pose2d pose) {
    return getDistanceTo(pose.getTranslation());
  }

  public double getDistanceTo(Translation3d translation) {
    return getDistanceTo(translation.toTranslation2d());
  }

  public double getDistanceTo(Translation2d translation) {
    Translation2d currentTranslation = getPose().getTranslation();
    return currentTranslation.getDistance(translation);
  }

  public double getHorizontalAngleTo(Pose2d pose) {
    return getHorizontalAngleTo(pose.getTranslation());
  }

  public double getHorizontalAngleTo(Translation3d translation) {
    return getHorizontalAngleTo(translation.toTranslation2d());
  }

  public double getHorizontalAngleTo(Translation2d translation) {
    Translation2d currentTranslation = getPose().getTranslation();
    double theta = currentTranslation.minus(translation).getAngle().getDegrees();
    return theta;
  }

  public double getVerticalAngleTo(Translation3d translation) {
    double horizontalDiff = getDistanceTo(translation);
    double zDiff = translation.getZ();
    double theta = Math.toDegrees(Math.atan2(zDiff, horizontalDiff));
    return theta;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the current odometry translation. */
  public Translation2d getTranslation() {
    return getPose().getTranslation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(lastGyroAngle, lastModulePositions, pose);
  }
}
