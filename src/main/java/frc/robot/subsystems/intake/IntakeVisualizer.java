// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class IntakeVisualizer {
  private final Mechanism2d mechanism;
  private final MechanismLigament2d intake;
  private final String key;

  private LoggedTunableNumber intakeOriginX = new LoggedTunableNumber("Intake/IntakeOriginX", 0);
  private LoggedTunableNumber intakeOriginY = new LoggedTunableNumber("Intake/IntakeOriginY", 0);
  private LoggedTunableNumber intakeOriginZ = new LoggedTunableNumber("Intake/IntakeOriginZ", 0);

  public IntakeVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new Mechanism2d(3.0, 3.0, new Color8Bit(Color.kBlack));
    MechanismRoot2d root = mechanism.getRoot("pivot", 1.0, 0.4);
    intake = new MechanismLigament2d("intake", intakeLength, 20.0, 6, new Color8Bit(color));
    root.append(intake);
  }

  /** Update intake visualizer with current intake angle */
  public void update(double angleRots) {
    // Log Mechanism2d
    double angleRads = Units.rotationsToRadians(-angleRots) / 100;
    intake.setAngle(Rotation2d.fromRadians(angleRads));
    Logger.recordOutput("Intake/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    Pose3d pivot =
        new Pose3d(
            0, intakeOrigin.getX(), intakeOrigin.getY(), new Rotation3d(-angleRads, 0.0, 0.0));
    Logger.recordOutput("Intake/Mechanism3d/" + key, pivot);
  }
}
