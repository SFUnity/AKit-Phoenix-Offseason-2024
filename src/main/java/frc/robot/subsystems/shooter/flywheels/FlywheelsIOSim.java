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

package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelsIOSim implements FlywheelsIO {
  private FlywheelSim simTop = new FlywheelSim(DCMotor.getNEO(1), 1.5, 0.004);
  private FlywheelSim simBottom = new FlywheelSim(DCMotor.getNEO(1), 1.5, 0.004);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(
              pid.calculate(simTop.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      simTop.setInputVoltage(appliedVolts);
    }

    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(
              pid.calculate(simBottom.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      simBottom.setInputVoltage(appliedVolts);
    }

    simTop.update(0.02);
    simBottom.update(0.02);

    inputs.velocityRadPerSecTop = simTop.getAngularVelocityRadPerSec();
    inputs.appliedVoltsTop = appliedVolts;
    inputs.currentAmpsTop = new double[] {simTop.getCurrentDrawAmps()};

    inputs.velocityRadPerSecBottom = simBottom.getAngularVelocityRadPerSec();
    inputs.appliedVoltsBottom = appliedVolts;
    inputs.currentAmpsBottom = new double[] {simBottom.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltageTop(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    simTop.setInputVoltage(volts);
  }

  @Override
  public void setVoltageBottom(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    simBottom.setInputVoltage(volts);
  }

  @Override
  public void setVelocityTop(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    pid.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  @Override
  public void setVelocityBottom(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    pid.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  @Override
  public void stopBoth() {
    setVoltageTop(0.0);
    setVoltageBottom(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
