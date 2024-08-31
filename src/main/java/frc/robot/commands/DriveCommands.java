package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.DriveCommandsConfig;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.05;
  private Drive drive;
  private PoseManager poseManager;
  private DriveCommandsConfig config;

  private static final LoggedTunableNumber linearkP =
      new LoggedTunableNumber("DriveCommands/Linear/kP", 3.5);
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("DriveCommands/Linear/kD", 0.0);
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("DriveCommands/Theta/kP", 6.0);
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("DriveCommands/Theta/D", 0.0);
  private static final LoggedTunableNumber linearTolerance =
      new LoggedTunableNumber("DriveCommands/Linear/controllerTolerance", 0.08);
  private static final LoggedTunableNumber thetaToleranceDeg =
      new LoggedTunableNumber("DriveCommands/Theta/controllerToleranceDeg", 1.0);

  private static final LoggedTunableNumber maxLinearVelocity =
      new LoggedTunableNumber(
          "DriveCommands/Linear/maxVelocity", DriveConstants.MAX_LINEAR_VELOCITY);
  private static final LoggedTunableNumber maxLinearAcceleration =
      new LoggedTunableNumber(
          "DriveCommands/Linear/maxAcceleration", DriveConstants.MAX_LINEAR_ACCELERATION * 0.4);
  private static final LoggedTunableNumber maxAngularVelocity =
      new LoggedTunableNumber(
          "DriveCommands/Theta/maxVelocity", DriveConstants.MAX_ANGULAR_VELOCITY * 0.8);
  private static final LoggedTunableNumber maxAngularAcceleration =
      new LoggedTunableNumber(
          "DriveCommands/Theta/maxAcceleration", DriveConstants.MAX_ANGULAR_ACCELERATION * 0.8);

  private final ProfiledPIDController thetaController;
  private final ProfiledPIDController linearController;

  public DriveCommands(Drive drive, PoseManager poseManager, DriveCommandsConfig config) {
    this.drive = drive;
    this.poseManager = poseManager;
    this.config = config;

    linearController =
        new ProfiledPIDController(
            linearkP.get(), 0, linearkD.get(), new TrapezoidProfile.Constraints(0, 0));
    linearController.setTolerance(linearTolerance.get());

    thetaController =
        new ProfiledPIDController(
            thetakP.get(), 0, thetakD.get(), new TrapezoidProfile.Constraints(0.0, 0.0));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(thetaToleranceDeg.get()));

    updateConstraints();
  }

  private void updateConstraints() {
    linearController.setConstraints(
        new TrapezoidProfile.Constraints(maxLinearVelocity.get(), maxLinearAcceleration.get()));
    updateThetaConstraints();
  }

  private void updateThetaConstraints() {
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(maxAngularVelocity.get(), maxAngularAcceleration.get()));
  }

  private void resetControllers(Pose2d goalPose) {
    Twist2d fieldVelocity = poseManager.fieldVelocity();
    double linearVelocity =
        Math.min(
            0.0,
            new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
                .rotateBy(poseManager.getHorizontalAngleTo(goalPose))
                .getX());
    linearController.reset(poseManager.getDistanceTo(goalPose), linearVelocity);
    resetThetaController();
  }

  private void resetThetaController() {
    Pose2d currentPose = poseManager.getPose();
    Twist2d fieldVelocity = poseManager.fieldVelocity();
    thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.dtheta);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public Command joystickDrive() {
    return Commands.run(
            () -> {
              // Convert to doubles
              double o = config.omegaJoystick().getAsDouble();

              // Check for slow mode
              if (config.slowMode().getAsBoolean()) {
                o *= config.slowTurnMultiplier().get();
              }

              // Apply deadband
              double omega = MathUtil.applyDeadband(o, DEADBAND);

              // Square values and scale to max velocity
              omega = Math.copySign(omega * omega, omega);
              omega *= DriveConstants.MAX_ANGULAR_VELOCITY;

              // Get linear velocity
              Translation2d linearVelocity = getLinearVelocityFromJoysticks();

              // Convert to field relative speeds & send command
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      linearVelocity.getX(),
                      linearVelocity.getY(),
                      omega,
                      AllianceFlipUtil.shouldFlip()
                          ? poseManager.getRotation().plus(new Rotation2d(Math.PI))
                          : poseManager.getRotation()));
            },
            drive)
        .withName("Joystick Drive");
  }

  /**
   * Field relative drive command using one joystick (controlling linear velocity) with a
   * ProfiledPID for angular velocity.
   */
  public Command headingDrive(Supplier<Rotation2d> goalHeading) {
    return Commands.run(
            () -> {
              updateThetaTunables();
              updateThetaConstraints();

              // Get linear velocity
              Translation2d linearVelocity = getLinearVelocityFromJoysticks();

              // Convert to field relative speeds & send command
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      linearVelocity.getX(),
                      linearVelocity.getY(),
                      getAngularVelocityFromProfiledPID(goalHeading.get().getRadians()),
                      AllianceFlipUtil.shouldFlip()
                          ? poseManager.getRotation().plus(new Rotation2d(Math.PI))
                          : poseManager.getRotation()));

              Leds.getInstance().alignedWithTarget = thetaAtGoal();
            },
            drive)
        .beforeStarting(
            () -> {
              resetThetaController();
            })
        .finallyDo(() -> Leds.getInstance().alignedWithTarget = false)
        .withName("Heading Drive");
  }

  /**
   * Field relative drive command using a ProfiledPID for linear velocity and a ProfiledPID for
   * angular velocity.
   */
  public Command fullAutoDrive(Supplier<Pose2d> goalPose) {
    return Commands.run(
            () -> {
              updateTunables();
              updateConstraints();

              // Calculate linear speed
              Pose2d targetPose = goalPose.get();

              double currentDistance = poseManager.getDistanceTo(targetPose);

              double driveVelocityScalar = linearController.calculate(currentDistance, 0.0);

              if (linearAtGoal()) driveVelocityScalar = 0.0;

              // Calculate angle to target then transform by velocity scalar
              Rotation2d angleToTarget = poseManager.getHorizontalAngleTo(targetPose);

              Translation2d driveVelocity = new Translation2d(driveVelocityScalar, angleToTarget);

              // Calculate theta speed
              double thetaVelocity =
                  getAngularVelocityFromProfiledPID(targetPose.getRotation().getRadians());
              if (thetaController.atGoal()) thetaVelocity = 0.0;

              // Send command
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      driveVelocity.getX(),
                      driveVelocity.getY(),
                      thetaVelocity,
                      poseManager.getRotation()));

              Leds.getInstance().alignedWithTarget = linearAtGoal() && thetaAtGoal();

              Logger.recordOutput("DriveCommands/Linear/currentDistance", currentDistance);
            },
            drive)
        .beforeStarting(
            () -> {
              resetControllers(goalPose.get());
            })
        .finallyDo(() -> Leds.getInstance().alignedWithTarget = false)
        .withName("Full Auto Drive");
  }

  private Translation2d getLinearVelocityFromJoysticks() {
    // Convert to doubles
    double x = config.xJoystick().getAsDouble();
    double y = config.yJoystick().getAsDouble();

    // The speed value here might need to change
    double povMovementSpeed = 0.5;
    if (config.povDown().getAsBoolean()) {
      x = -povMovementSpeed;
    } else if (config.povUp().getAsBoolean()) {
      x = povMovementSpeed;
    } else if (config.povLeft().getAsBoolean()) {
      y = povMovementSpeed;
    } else if (config.povRight().getAsBoolean()) {
      y = -povMovementSpeed;
    }

    // Check for slow mode
    if (config.slowMode().getAsBoolean()) {
      double multiplier = config.slowDriveMultiplier().get();
      x *= multiplier;
      y *= multiplier;
    }

    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(x, y);

    // Square values and scale to max velocity
    linearMagnitude = linearMagnitude * linearMagnitude;
    linearMagnitude *= DriveConstants.MAX_LINEAR_VELOCITY;

    // Calcaulate new linear velocity
    Translation2d linearVelocity = new Translation2d(linearMagnitude, linearDirection);

    return linearVelocity;
  }

  private double getAngularVelocityFromProfiledPID(double goalHeadingRads) {
    double output =
        thetaController.calculate(
            poseManager.getPose().getRotation().getRadians(), goalHeadingRads);

    Logger.recordOutput("DriveCommands/Theta/HeadingError", thetaController.getPositionError());
    return output;
  }

  private void updateTunables() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> linearController.setPID(linearkP.get(), 0, linearkD.get()),
        linearkP,
        linearkD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> linearController.setTolerance(linearTolerance.get()), linearTolerance);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> linearController.setPID(linearkP.get(), 0, linearkD.get()),
        linearkP,
        linearkD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> linearController.setTolerance(linearTolerance.get()), linearTolerance);

    LoggedTunableNumber.ifChanged(
        hashCode(), this::updateConstraints, maxLinearVelocity, maxLinearAcceleration);
    updateThetaTunables();
  }

  private void updateThetaTunables() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> thetaController.setPID(thetakP.get(), 0, thetakD.get()),
        thetakP,
        thetakD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> thetaController.setTolerance(Units.degreesToRadians(thetaToleranceDeg.get())),
        thetaToleranceDeg);
    LoggedTunableNumber.ifChanged(
        hashCode(), this::updateThetaConstraints, maxAngularVelocity, maxAngularAcceleration);
  }

  /** Returns true if within tolerance of aiming at goal */
  @AutoLogOutput(key = "DriveCommands/Linear/AtGoal")
  public boolean linearAtGoal() {
    return linearController.atGoal();
  }

  /** Returns true if within tolerance of aiming at speaker */
  @AutoLogOutput(key = "DriveCommands/Theta/AtGoal")
  public boolean thetaAtGoal() {
    return EqualsUtil.equalsWithTolerance(
        thetaController.getSetpoint().position,
        thetaController.getGoal().position,
        Units.degreesToRadians(thetaToleranceDeg.get()));
  }
}
