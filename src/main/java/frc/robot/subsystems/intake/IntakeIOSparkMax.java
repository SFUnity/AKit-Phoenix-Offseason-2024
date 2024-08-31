package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class IntakeIOSparkMax implements IntakeIO {
  private static final double autoStartAngle = Units.degreesToRadians(0.0);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          100,
          1.06328,
          intakeLength,
          Math.toRadians(0.0),
          Math.toRadians(125.0),
          false,
          Units.degreesToRadians(0.0));

  private final PIDController controller;
  private double pivotAppliedVoltage = 0.0;

  private double rollersAppliedVoltage = 0.0;
  private double indexerAppliedVoltage = 0.0;

  private boolean controllerNeedsReset = false;
  private boolean wasNotAuto = true;

  public IntakeIOSparkMax() {
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

    inputs.pivotPositionRots = Units.radiansToRotations(sim.getAngleRads()) * 100;
    inputs.pivotAppliedVolts = pivotAppliedVoltage;
    inputs.pivotCurrentAmps = sim.getCurrentDrawAmps();

    inputs.rollersAppliedVolts = rollersAppliedVoltage;
    inputs.indexerAppliedVolts = indexerAppliedVoltage;

    // Reset input
    sim.setInputVoltage(0.0);
  }

  @Override
  public void runIntakeRollers(double percentOutput) {
    rollersAppliedVoltage = 12 * percentOutput;
  }

  @Override
  public void runIndexer(double percentOutput) {
    indexerAppliedVoltage = 12 * percentOutput;
  }

  @Override
  public void setPivotPosition(double setpointRots) {
    if (controllerNeedsReset) {
      controller.reset();
      controllerNeedsReset = false;
    }

    double setpointRads = Units.rotationsToRadians(setpointRots) / 100;
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
