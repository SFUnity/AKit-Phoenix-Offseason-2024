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

import static frc.robot.subsystems.intake.IntakeConstants.angleMotorId;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class PivotIOSparkMax implements PivotIO {
  private final CANSparkMax angleMotor = new CANSparkMax(angleMotorId, MotorType.kBrushless);
  private final RelativeEncoder encoder = angleMotor.getEncoder();
  private final SparkPIDController pid = angleMotor.getPIDController();

  public PivotIOSparkMax() {
    angleMotor.restoreFactoryDefaults();

    angleMotor.setCANTimeout(250);

    angleMotor.setInverted(false);

    angleMotor.enableVoltageCompensation(12.0);
    angleMotor.setSmartCurrentLimit(30);

    angleMotor.burnFlash();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionRots = encoder.getPosition();
    inputs.velocityRotsPerSec = encoder.getVelocity();
    inputs.appliedVolts = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
    inputs.currentAmps =
        new double[] {angleMotor.getOutputCurrent(), angleMotor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    angleMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    angleMotor.stopMotor();
  }

  @Override
  public void setAngleMotorSpeeds(double desiredAngle) {
    pid.setReference(desiredAngle, ControlType.kPosition);
  }

  @Override
  public void setP(double kP) {
    pid.setP(kP, 0);
    pid.setI(0, 0);
    pid.setD(0, 0);
    pid.setFF(0, 0);
  }
}
