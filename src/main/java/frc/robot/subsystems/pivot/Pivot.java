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

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.pivot.PivotConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final AprilTagVision aprilTagVision;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;
  private ShuffleboardTab driversTab = Shuffleboard.getTab("Drivers"); // TODO change these to LoggedDashboardNumbers

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

  /** Creates a new Flywheel. */
  public Pivot(PivotIO io, AprilTagVision aprilTagVision) {
    this.io = io;

    this.aprilTagVision = aprilTagVision;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Pivot/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  // TODO Is this used anywhere? Also think about units of desiredAngle vs positionRad
  public boolean atDesiredAngle(double desiredAngle) {
    return inputs.positionRots <= desiredAngle + 1 || inputs.positionRots >= desiredAngle - 1;
  }

  // TODO Do we need a function to control the pivot using velocity control?
  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Pivot/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  // TODO Does your code need this conversion? If not AScope can do it for you, so I would suggest removing
  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRotsPerSec);
  }

  // TODO After a bit of research I don't think this is necessary and will be removed everywhere (left it here for you to delete bc this is your subsystem)
  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRotsPerSec;
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

  public Command setManualShootAngleCommand() {
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

  public Command setAutoShootAngleCommand() {
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

  public Command setAmpAngleCommand() {
    return run(() -> {
          readyShootAmp();
          io.setAngleMotorSpeeds(desiredAngle);
        })
        .withName("setAmpAngle");
  }

  public Command setFeedAngleCommand() {
    return run(() -> {
          readyShootFeed();
          io.setAngleMotorSpeeds(desiredAngle);
        })
        .finallyDo(
            () -> {
              readyShootAmp();
              io.setAngleMotorSpeeds(desiredAngle);
            })
        .withName("gotta be a team player"); // TODO lol :D (please delete me after reading)
  }
}
