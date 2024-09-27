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

import static frc.robot.subsystems.shooter.pivot.PivotConstants.pivotLength;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
  private static final double autoStartAngle = Units.degreesToRadians(0.0);
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          100,
          1.06328,
          pivotLength,
          Math.toRadians(0.0),
          Math.toRadians(125.0),
          false,
          Units.degreesToRadians(0.0));

  private final PIDController pid;
  private double appliedVolts = 0.0;
  private double desiredAngle = 0.0;

  private boolean controllerNeedsReset = false;
  private boolean wasNotAuto = true;

  public PivotIOSim() {
    pid = new PIDController(0.0, 0.0, 0.0);
    sim.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      controllerNeedsReset = true;
    }

    // Reset at start of auto
    if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
      sim.setState(autoStartAngle, 0.0);
      wasNotAuto = false;
    }
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    sim.update(0.02);

    inputs.positionRots = desiredAngle;
    inputs.velocityRotsPerSec = Units.radiansToRotations(sim.getVelocityRadPerSec());
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setAngleMotor(double desiredAngle) {
    this.desiredAngle = desiredAngle;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setP(double kP) {
    pid.setPID(kP, 0, 0);
  }
}
