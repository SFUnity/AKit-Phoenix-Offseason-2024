// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMixed;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.BeamBreakIO;
import frc.robot.subsystems.shooter.BeamBreakIORev;
import frc.robot.subsystems.shooter.BeamBreakIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIO;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIOSim;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIOSparkMax;
import frc.robot.subsystems.shooter.pivot.Pivot;
import frc.robot.subsystems.shooter.pivot.PivotIO;
import frc.robot.subsystems.shooter.pivot.PivotIOSim;
import frc.robot.subsystems.shooter.pivot.PivotIOSparkMax;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final AprilTagVision aprilTagVision;
  private final Shooter shooter;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandPS5Controller operator = new CommandPS5Controller(1);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOMixed(0),
                new ModuleIOMixed(1),
                new ModuleIOMixed(2),
                new ModuleIOMixed(3));
        intake = new Intake(new IntakeIOSim());
        aprilTagVision = new AprilTagVision(new AprilTagVisionIOLimelight("limelight"));
        shooter =
            new Shooter(
                new Flywheels(new FlywheelsIOSparkMax()),
                new Pivot(new PivotIOSparkMax(), aprilTagVision),
                new BeamBreakIORev());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = new Intake(new IntakeIOSim());
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
        shooter =
            new Shooter(
                new Flywheels(new FlywheelsIOSim()),
                new Pivot(new PivotIOSim(), aprilTagVision),
                new BeamBreakIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new Intake(new IntakeIO() {});
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
        shooter =
            new Shooter(
                new Flywheels(new FlywheelsIO() {}),
                new Pivot(new PivotIO() {}, aprilTagVision),
                new BeamBreakIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    // Alerts for constants
    if (Constants.tuningMode) {
      new Alert("Tuning mode enabled", AlertType.INFO).set(true);
    }

    SmartDashboard.putData(
        intake //        this ↓↓ is an annoying workaround, but better than the alternatives IMO
            .intakeCmd(new Trigger(() -> true))
            .withTimeout(5)
            .withName("Lower and Run Intake"));
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // Default cmds
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    intake.setDefaultCommand(intake.raiseAndStopCmd());

    // Driver controls
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driver
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Operator controls for intake
    operator.triangle().whileTrue(intake.poopCmd());
    operator.square().whileTrue(intake.intakeCmd(operator.cross()));
    operator.circle().whileTrue(intake.intakeCmd(new Trigger(() -> false)));

    // Operator controls for pivot
    operator.circle().whileTrue(shooter.setManualAngle());
  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(
                driver.getHID().getPort())); // Should be an XBox controller
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operator.getHID().getPort())
            || DriverStation.getJoystickIsXbox(
                operator.getHID().getPort())); // Should not be an XBox controller
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
