// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.EqualsUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class DriveCommands {
  private static final double DEADBAND = 0.05;
  private Drive drive;
  private DoubleSupplier xJoystickIn;
  private DoubleSupplier yJoystickIn;
  private DoubleSupplier omegaJoystickIn;
  private BooleanSupplier fastMode;
  private LoggedDashboardNumber slowDriveMultiplier;
  private LoggedDashboardNumber slowTurnMultiplier;
  private Trigger povUp;
  private Trigger povDown;
  private Trigger povLeft;
  private Trigger povRight;
  private PoseManager poseManager;

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

  public DriveCommands(
      Drive drive,
      DoubleSupplier xJoystickIn,
      DoubleSupplier yJoystickIn,
      DoubleSupplier omegaJoystickIn,
      BooleanSupplier fastMode,
      LoggedDashboardNumber slowDriveMultiplier,
      LoggedDashboardNumber slowTurnMultiplier,
      Trigger povUp,
      Trigger povDown,
      Trigger povLeft,
      Trigger povRight,
      PoseManager poseManager) {
    this.drive = drive;
    this.xJoystickIn = xJoystickIn;
    this.yJoystickIn = yJoystickIn;
    this.omegaJoystickIn = omegaJoystickIn;
    this.fastMode = fastMode;
    this.slowDriveMultiplier = slowDriveMultiplier;
    this.slowTurnMultiplier = slowTurnMultiplier;
    this.povUp = povUp;
    this.povDown = povDown;
    this.povLeft = povLeft;
    this.povRight = povRight;
    this.poseManager = poseManager;

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
              double o = omegaJoystickIn.getAsDouble();

              // Check for slow mode
              if (fastMode.getAsBoolean()) {
                o *= slowTurnMultiplier.get();
              }

              // Apply deadband
              double omega = MathUtil.applyDeadband(o, DEADBAND);

              // Square values
              omega = Math.copySign(omega * omega, omega);

              // Get linear velocity
              Translation2d linearVelocity = getLinearVelocityFromJoysticks();

              // Convert to field relative speeds & send command
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      linearVelocity.getX() * DriveConstants.MAX_LINEAR_VELOCITY,
                      linearVelocity.getY() * DriveConstants.MAX_LINEAR_VELOCITY,
                      omega * DriveConstants.MAX_ANGULAR_VELOCITY,
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
                      linearVelocity.getX() * DriveConstants.MAX_LINEAR_VELOCITY,
                      linearVelocity.getY() * DriveConstants.MAX_LINEAR_VELOCITY,
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

              Translation2d driveVelocity =
                  new Pose2d(new Translation2d(), angleToTarget)
                      .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
                      .getTranslation();

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
    double x = xJoystickIn.getAsDouble();
    double y = yJoystickIn.getAsDouble();

    // The speed value here might need to change
    double povMovementSpeed = 0.5;
    if (povDown.getAsBoolean()) {
      x = -povMovementSpeed;
    } else if (povUp.getAsBoolean()) {
      x = povMovementSpeed;
    } else if (povLeft.getAsBoolean()) {
      y = povMovementSpeed;
    } else if (povRight.getAsBoolean()) {
      y = -povMovementSpeed;
    }

    // Check for slow mode
    if (fastMode.getAsBoolean()) {
      double multiplier = slowDriveMultiplier.get();
      x *= multiplier;
      y *= multiplier;
    }

    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(x, y);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

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
