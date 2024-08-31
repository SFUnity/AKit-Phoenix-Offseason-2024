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

package frc.robot.subsystems.shooter.pivot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final AprilTagVision aprilTagVision;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final PivotVisualizer measuredVisualizer;
  private final PivotVisualizer setpointVisualizer;
  private ShuffleboardTab driversTab = Shuffleboard.getTab("Drivers");

  private GenericEntry hegihtOfSpeakerEntry =
      driversTab
          .addPersistent("Speaker Height", AprilTagVisionConstants.heightOfSpeakerInches)
          .withPosition(9, 1)
          .withSize(1, 1)
          .getEntry();

  private GenericEntry feedingAngleEntry =
      driversTab
          .addPersistent("Feeding Angle", PivotConstants.kFeedingAngleRevRotations)
          .withPosition(8, 0)
          .withSize(1, 1)
          .getEntry();

  private GenericEntry angleOffset =
      driversTab
          .addPersistent("Angle Offset", PivotConstants.kSpeakerAngleOffsetRevRotations)
          .withPosition(8, 1)
          .withSize(1, 1)
          .getEntry();

  private double desiredAngle = 0;

  /** Creates a new Pivot. */
  public Pivot(PivotIO io, AprilTagVision aprilTagVision) {
    this.io = io;

    this.aprilTagVision = aprilTagVision;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        break;
    }

    measuredVisualizer = new PivotVisualizer("Measured", Color.kRed);
    setpointVisualizer = new PivotVisualizer("Setpoint", Color.kBlue);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Pivot", inputs);

    measuredVisualizer.update(inputs.positionRots);
    setpointVisualizer.update(desiredAngle);
    Logger.recordOutput("Shooter/Pivot/positionSetpointRots", desiredAngle);
    GeneralUtil.logSubsystem(this, "Shooter/Pivot");
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Stops the pivot. */
  public void stop() {
    io.stop();
  }

  public void readyShootSpeakerManual() {
    desiredAngle = PivotConstants.kSpeakerManualAngleRevRotations;
  }

  public void readyShootSpeakerAutomatic() {
    double heightOfTarget =
        hegihtOfSpeakerEntry.getDouble(AprilTagVisionConstants.heightOfSpeakerInches);
    double angleRad = Math.atan(heightOfTarget / aprilTagVision.getDistance());
    double angleDeg = Math.toDegrees(angleRad);
    desiredAngle = angleDeg + angleOffset.getDouble(PivotConstants.kSpeakerAngleOffsetRevRotations);
  }

  public void readyShootAmp() {
    desiredAngle = PivotConstants.kDesiredAmpAngleRevRotations;
  }

  public void readyShootFeed() {
    desiredAngle = feedingAngleEntry.getDouble(PivotConstants.kFeedingAngleRevRotations);
  }

  public void readyShooterIntake() {
    desiredAngle = PivotConstants.kDesiredIntakeAngleRevRotations;
  }

  public void readyShooterEject() {
    desiredAngle = PivotConstants.kDesiredEjectAngleRevRotations;
  }

  public boolean atDesiredAngle() {
    return EqualsUtil.equalsWithTolerance(inputs.positionRots, desiredAngle, 0.5);
  }

  // TODO any setter methods used in these commands should be made private
  public Command setManualSpeakerAngle() {
    return run(() -> {
          readyShootSpeakerManual();
          io.setAngleMotorSpeeds(desiredAngle);
        })
        .finallyDo(
            () -> {
              readyShootAmp();
              io.setAngleMotorSpeeds(desiredAngle);
            })
        .withName("setManualShootAngle");
  }

  public Command setAutoSpeakerAngle() {
    return run(() -> {
          readyShootSpeakerAutomatic();
          io.setAngleMotorSpeeds(desiredAngle);
        })
        .finallyDo(
            () -> {
              readyShootAmp();
              io.setAngleMotorSpeeds(desiredAngle);
            })
        .withName("setAutoShootAngle");
  }

  public Command setAmpAngle() {
    return run(() -> {
          readyShootAmp();
          io.setAngleMotorSpeeds(desiredAngle);
        })
        .withName("setAmpAngle");
  }

  public Command setFeedAngle() {
    return run(() -> {
          readyShootFeed();
          io.setAngleMotorSpeeds(desiredAngle);
        })
        .finallyDo(
            () -> {
              readyShootAmp();
              io.setAngleMotorSpeeds(desiredAngle);
            })
        .withName("gotta be a team player");
  }

  public Command setIntakeAngle() {
    return run(() -> {
          readyShooterIntake();
          io.setAngleMotorSpeeds(desiredAngle);
        })
        .finallyDo(
            () -> {
              readyShootAmp();
              io.setAngleMotorSpeeds(desiredAngle);
            })
        .withName("setIntakeAngle");
  }
}
