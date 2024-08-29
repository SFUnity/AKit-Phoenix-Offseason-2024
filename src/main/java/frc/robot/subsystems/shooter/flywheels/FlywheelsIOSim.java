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

  private double appliedVoltsTop = 0.0;
  private double appliedVoltsBottom = 0.0;

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    if (closedLoop) {
      appliedVoltsTop =
          MathUtil.clamp(
              pid.calculate(simTop.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      simTop.setInputVoltage(appliedVoltsTop);
    }

    if (closedLoop) {
      appliedVoltsBottom =
          MathUtil.clamp(
              pid.calculate(simBottom.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      simBottom.setInputVoltage(appliedVoltsBottom);
    }

    simTop.update(0.02);
    simBottom.update(0.02);

    inputs.velocityRadPerSecTop = simTop.getAngularVelocityRadPerSec();
    inputs.appliedVoltsTop = appliedVoltsTop;
    inputs.currentAmpsTop = new double[] {simTop.getCurrentDrawAmps()};

    inputs.velocityRadPerSecBottom = simBottom.getAngularVelocityRadPerSec();
    inputs.appliedVoltsBottom = appliedVoltsBottom;
    inputs.currentAmpsBottom = new double[] {simBottom.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltageTop(double volts) {
    closedLoop = false;
    appliedVoltsTop = volts;
    simTop.setInputVoltage(volts);
  }

  @Override
  public void setVoltageBottom(double volts) {
    closedLoop = false;
    appliedVoltsBottom = volts;
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
