package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax angleMotor = new CANSparkMax(angleMotorId, MotorType.kBrushless);
  private final RelativeEncoder encoder = angleMotor.getEncoder();
  private final SparkPIDController pid = angleMotor.getPIDController();
  private final CANSparkMax rollersMotor = new CANSparkMax(rollersMotorId, MotorType.kBrushless);
  private final CANSparkMax handoffMotor = new CANSparkMax(handoffMotorId, MotorType.kBrushless);

  public IntakeIOSparkMax() {
    basicMotorConfig(angleMotor);
    basicMotorConfig(rollersMotor);
    basicMotorConfig(handoffMotor);
  }

  private void basicMotorConfig(CANSparkMax motor) {
    motor.restoreFactoryDefaults();

    motor.setCANTimeout(250);

    motor.setInverted(false);

    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(30);

    motor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivotPositionRots = encoder.getPosition();
    inputs.pivotAppliedVolts = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
    inputs.pivotCurrentAmps = angleMotor.getOutputCurrent();

    inputs.rollersAppliedVolts = rollersMotor.getAppliedOutput();
    inputs.indexerAppliedVolts = handoffMotor.getAppliedOutput();
  }

  @Override
  public void runIntakeRollers(double percentOutput) {
    rollersMotor.set(percentOutput);
  }

  @Override
  public void runIndexer(double percentOutput) {
    handoffMotor.set(percentOutput);
  }

  @Override
  public void setPivotPosition(double setpointRots) {
    pid.setReference(setpointRots, ControlType.kPosition);
  }

  @Override
  public void setP(double p) {
    pid.setP(p);
  }
}
