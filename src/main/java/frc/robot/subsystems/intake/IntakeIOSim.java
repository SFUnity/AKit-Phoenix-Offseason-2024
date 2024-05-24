package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim {
    private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          100,
          1.06328,
          Units.inchesToMeters(13.835),
          Math.toRadians(0.0),
          Math.toRadians(125.0),
          false,
          Units.degreesToRadians(0.0));
}
