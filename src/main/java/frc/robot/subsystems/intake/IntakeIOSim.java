package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private static final double autoStartAngle = Units.degreesToRadians(0.0);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          100,
          1.06328,
          Units.inchesToMeters(13.835),
          Math.toRadians(0.0),
          Math.toRadians(125.0),
          false,
          Units.degreesToRadians(0.0));

  private final PIDController controller;
  private double pivotAppliedVoltage = 0.0;

  private boolean controllerNeedsReset = false;
  private boolean wasNotAuto = true;

  public IntakeIOSim() {
    controller = new PIDController(0.0, 0.0, 0.0);
    sim.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      controllerNeedsReset = true;
    }

    // Reset at start of auto
    if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
      sim.setState(autoStartAngle, 0.0);
      wasNotAuto = false;
    }
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    sim.update(Constants.loopPeriodSecs);

    inputs.pivotPositionRad = Units.radiansToRotations(sim.getAngleRads()) * 100;
    inputs.pivotAppliedVolts = pivotAppliedVoltage;
    inputs.pivotCurrentAmps = sim.getCurrentDrawAmps(); // TODO WPILib spelling issue

    // Reset input
    sim.setInputVoltage(0.0);
  }

  @Override
  public void setPivotPosition(double setpointRads) {
    if (controllerNeedsReset) {
      controller.reset();
      controllerNeedsReset = false;
    }

    double volts = controller.calculate(sim.getAngleRads(), setpointRads);
    pivotAppliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(pivotAppliedVoltage);
  }

  @Override
  public void setPID(double p) {
    controller.setPID(p, 0, 0);
  }

  @Override
  public void stop() {
    pivotAppliedVoltage = 0.0;
    sim.setInputVoltage(pivotAppliedVoltage);
  }
}
