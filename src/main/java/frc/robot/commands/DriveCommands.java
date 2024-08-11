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

import static frc.robot.subsystems.drive.DriveConstants.headingControllerConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.EqualsUtil;
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

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("HeadingController/kP", headingControllerConstants.kP());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("HeadingController/kD", headingControllerConstants.kD());
  private static final LoggedTunableNumber maxVelocityMultipler =
      new LoggedTunableNumber("HeadingController/MaxVelocityMultipler", 0.8);
  private static final LoggedTunableNumber maxAccelerationMultipler =
      new LoggedTunableNumber("HeadingController/MaxAccelerationMultipler", 0.8);
  private static final LoggedTunableNumber toleranceDegrees =
      new LoggedTunableNumber("HeadingController/ToleranceDegrees", 1.0);

  private final ProfiledPIDController controller;
  private Supplier<Rotation2d> goalHeadingSupplier;

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

    controller =
        new ProfiledPIDController(
            kP.get(),
            0,
            kD.get(),
            new TrapezoidProfile.Constraints(0.0, 0.0),
            Constants.loopPeriodSecs);
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));

    controller.reset(
        poseManager.getPose().getRotation().getRadians(), poseManager.fieldVelocity().dtheta);
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
          Translation2d linearVelocity = getLinearVelocityFromJoysticks();

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
  public Command headingDrive(Supplier<Rotation2d> goalHeadingSupplier) {
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity = getLinearVelocityFromJoysticks();

              // Convert to field relative speeds & send command
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED,
                      linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED,
                      getAngularVelocityFromProfiledPID(),
                      AllianceFlipUtil.shouldFlip()
                          ? poseManager.getRotation().plus(new Rotation2d(Math.PI))
                          : poseManager.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              controller.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));
              this.goalHeadingSupplier = goalHeadingSupplier;

              controller.reset(
                  poseManager.getPose().getRotation().getRadians(),
                  poseManager.fieldVelocity().dtheta);
            });
  }

  private Translation2d getLinearVelocityFromJoysticks() {
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

  private double getAngularVelocityFromProfiledPID() {
    // Update controller
    controller.setPID(kP.get(), 0, kD.get());
    controller.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));

    double maxAngularAcceleration =
        DriveConstants.MAX_ANGULAR_ACCELERATION * maxAccelerationMultipler.get();
    double maxAngularVelocity = DriveConstants.MAX_ANGULAR_SPEED * maxVelocityMultipler.get();
    controller.setConstraints(
        new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration));

    double output =
        controller.calculate(
            poseManager.getPose().getRotation().getRadians(),
            goalHeadingSupplier.get().getRadians());

    Logger.recordOutput("Drive/HeadingController/HeadingError", controller.getPositionError());
    return output;
  }

  /** Returns true if within tolerance of aiming at goal */
  @AutoLogOutput(key = "Drive/HeadingController/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.equalsWithTolerance(
        controller.getSetpoint().position,
        controller.getGoal().position,
        Units.degreesToRadians(toleranceDegrees.get()));
  }
}
