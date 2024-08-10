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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.PoseManager;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class DriveCommands {
  private static final double DEADBAND = 0.05;
  private Drive drive;
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;
  private DoubleSupplier omegaSupplier;
  private BooleanSupplier fastMode;
  private LoggedDashboardNumber slowDriveMultiplier;
  private LoggedDashboardNumber slowTurnMultiplier;
  private Trigger povUp;
  private Trigger povDown;
  private Trigger povLeft;
  private Trigger povRight;
  private PoseManager poseManager;

  public DriveCommands(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier fastMode,
      LoggedDashboardNumber slowDriveMultiplier,
      LoggedDashboardNumber slowTurnMultiplier,
      Trigger povUp,
      Trigger povDown,
      Trigger povLeft,
      Trigger povRight,
      PoseManager poseManager) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.fastMode = fastMode;
    this.slowDriveMultiplier = slowDriveMultiplier;
    this.slowTurnMultiplier = slowTurnMultiplier;
    this.povUp = povUp;
    this.povDown = povDown;
    this.povLeft = povLeft;
    this.povRight = povRight;
    this.poseManager = poseManager;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public Command joystickDrive() {
    return Commands.run(
        () -> {
          // Convert to doubles
          double o = omegaSupplier.getAsDouble();

          // Check for slow mode
          if (fastMode.getAsBoolean()) {
            o *= slowTurnMultiplier.get();
          }

          // Apply deadband
          double omega = MathUtil.applyDeadband(o, DEADBAND);

          // Square values
          omega = Math.copySign(omega * omega, omega);

          // Get linear velocity
          Translation2d linearVelocity = getGoalLinearVelocityFromJoysticks();

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED,
                  linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED,
                  omega * DriveConstants.MAX_ANGULAR_SPEED,
                  AllianceFlipUtil.shouldFlip()
                      ? poseManager.getRotation().plus(new Rotation2d(Math.PI))
                      : poseManager.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using one joystick (controlling linear velocity) with a
   * ProfiledPID for angular velocity.
   */
  public Command headingDrive(DoubleSupplier desiredAngle) {
    return Commands.run(
        () -> {
          // Get angular velocity
          double omega = getGoalAngularVelocityFromProfiledPID();

          // Get linear velocity
          Translation2d linearVelocity = getGoalLinearVelocityFromJoysticks();

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED,
                  linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED,
                  omega * DriveConstants.MAX_ANGULAR_SPEED,
                  AllianceFlipUtil.shouldFlip()
                      ? poseManager.getRotation().plus(new Rotation2d(Math.PI))
                      : poseManager.getRotation()));
        },
        drive);
  }

  private Translation2d getGoalLinearVelocityFromJoysticks() {
    // Convert to doubles
    double x = xSupplier.getAsDouble();
    double y = ySupplier.getAsDouble();

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

  private double getGoalAngularVelocityFromProfiledPID() {
    return 0; // TODO make getGoalAngularVelocity work
  }
}
