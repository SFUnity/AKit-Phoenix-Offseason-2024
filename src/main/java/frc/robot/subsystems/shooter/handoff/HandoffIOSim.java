package frc.robot.subsystems.shooter.handoff;

public class HandoffIOSim implements HandoffIO {
  private double appliedVolts = 0.0;

  public HandoffIOSim() {}

  @Override
  public void updateInputs(HandoffIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}
