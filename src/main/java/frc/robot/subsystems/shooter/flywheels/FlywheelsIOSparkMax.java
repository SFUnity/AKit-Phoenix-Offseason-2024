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

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class FlywheelsIOSparkMax implements FlywheelsIO {
  private final CANSparkMax top = new CANSparkMax(9, MotorType.kBrushless);
  private final CANSparkMax bottom = new CANSparkMax(10, MotorType.kBrushless);
  private final RelativeEncoder encoderTop = top.getEncoder();
  private final RelativeEncoder encoderBottom = bottom.getEncoder();
  private final SparkPIDController pidTop = top.getPIDController();
  private final SparkPIDController pidBottom = bottom.getPIDController();

  public FlywheelsIOSparkMax() {
    top.restoreFactoryDefaults();
    bottom.restoreFactoryDefaults();

    top.setCANTimeout(250);
    bottom.setCANTimeout(250);

    top.setInverted(false);
    bottom.setInverted(true);

    top.enableVoltageCompensation(12.0);
    top.setSmartCurrentLimit(30);

    bottom.enableVoltageCompensation(12.0);
    bottom.setSmartCurrentLimit(30);

    top.burnFlash();
    bottom.burnFlash();
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.velocityRadPerSecTop =
        Units.rotationsPerMinuteToRadiansPerSecond(encoderTop.getVelocity());
    inputs.appliedVoltsTop = top.getAppliedOutput() * top.getBusVoltage();
    inputs.currentAmpsTop = new double[] {top.getOutputCurrent()};

    inputs.velocityRadPerSecBottom =
        Units.rotationsPerMinuteToRadiansPerSecond(encoderBottom.getVelocity());
    inputs.appliedVoltsBottom = bottom.getAppliedOutput() * bottom.getBusVoltage();
    inputs.currentAmpsBottom = new double[] {bottom.getOutputCurrent()};
  }

  @Override
  public void setVoltageTop(double volts) {
    top.setVoltage(volts);
  }

  @Override
  public void setVoltageBottom(double volts) {
    bottom.setVoltage(volts);
  }

  @Override
  public void setVelocityTop(double velocityRadPerSec, double ffVolts) {
    pidTop.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setVelocityBottom(double velocityRadPerSec, double ffVolts) {
    pidBottom.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stopBoth() {
    top.stopMotor();
    bottom.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pidTop.setP(kP, 0);
    pidTop.setI(kI, 0);
    pidTop.setD(kD, 0);
    pidTop.setFF(0, 0);

    pidBottom.setP(kP, 0);
    pidBottom.setI(kI, 0);
    pidBottom.setD(kD, 0);
    pidBottom.setFF(0, 0);
  }
}
