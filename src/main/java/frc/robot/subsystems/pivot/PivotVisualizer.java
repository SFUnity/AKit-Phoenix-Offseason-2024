// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class PivotVisualizer {
  private final Mechanism2d mechanism;
  private final MechanismLigament2d pivot;
  private final String key = "";

  private static final double angleOffset = 1.63;

  private static final double gearReduction = 73;

  public pivotVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new Mechanism2d(3.0, 3.0, new Color8Bit(Color.kBlack));
    MechanismRoot2d root = mechanism.getRoot("pivot", 1.5, 2.0);
    pivot =
        new MechanismLigament2d("pivot", PivotConstants.pivotLength, 20.0, 6, new Color8Bit(color));
    root.append(pivot);
  }

  /** Update intake visualizer with current intake angle */
  public void update(double angleRots) {
    // Log Mechanism2d
    double angleRads = (Units.rotationsToRadians(-angleRots)) / gearReduction + angleOffset;
    pivot.setAngle(Rotation2d.fromRadians(angleRads));
    Logger.recordOutput("Pivot/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    Pose3d pivot =
        new Pose3d(
            PivotConstants.pivotOrigin.getX(),
            0,
            PivotConstants.pivotOrigin.getY(),
            new Rotation3d(0, -angleRads, 0.0));
    Logger.recordOutput("Potvt/Mechanism3d/" + key, pivot);
  }
}
