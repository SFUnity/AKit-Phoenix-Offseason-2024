package frc.robot.subsystems.shooter.feeder;

public class FeederIOSparkMax implements FeederIO {
  private double appliedVolts = 0.0;

  public FeederIOSparkMax() {}

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void runPercent(double percent) {
    appliedVolts = percent * 12;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}
