package frc.robot.subsystems.shooter.feeder;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class FeederIOSparkMax implements FeederIO {
  private final CANSparkMax motor = new CANSparkMax(7, MotorType.kBrushless);

  public FeederIOSparkMax() {
    motor.restoreFactoryDefaults();

    motor.setCANTimeout(250);

    motor.setInverted(false);

    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(30);

    motor.burnFlash();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput();
  }

  @Override
  public void runPercent(double percent) {
    motor.set(percent);
  }

  @Override
  public void stop() {
    runPercent(0.0);
  }
}
