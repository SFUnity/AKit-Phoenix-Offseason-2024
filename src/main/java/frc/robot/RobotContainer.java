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

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.DriveCommandsConfig;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMixed;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.BeamBreakIO;
import frc.robot.subsystems.shooter.BeamBreakIORev;
import frc.robot.subsystems.shooter.BeamBreakIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.feeder.Feeder;
import frc.robot.subsystems.shooter.feeder.FeederIO;
import frc.robot.subsystems.shooter.feeder.FeederIOSim;
import frc.robot.subsystems.shooter.feeder.FeederIOSparkMax;
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
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.PoseManager;
import frc.robot.util.loggedShuffleboardClasses.LoggedShuffleboardChooser;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

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

  // Pose Manager
  private final PoseManager poseManager = new PoseManager();

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandPS5Controller operator = new CommandPS5Controller(1);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);
  public boolean slowMode = false;

  // Dashboard inputs
  private final LoggedShuffleboardChooser<Runnable> autoChooser =
      new LoggedShuffleboardChooser<Runnable>("Auto Chooser", "Driver")
          .withSize(2, 1)
          .withPosition(0, 0);
  private final LoggedDashboardNumber slowDriveMultiplier =
      new LoggedDashboardNumber("Slow Drive Multiplier", 0.6);
  private final LoggedDashboardNumber slowTurnMultiplier =
      new LoggedDashboardNumber("Slow Turn Multiplier", 0.5);

  private final DriveCommandsConfig driveCommandsConfig =
      new DriveCommandsConfig(driver, () -> slowMode, slowDriveMultiplier, slowTurnMultiplier);

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
                new ModuleIOMixed(3),
                poseManager,
                driveCommandsConfig);
        intake = new Intake(new IntakeIOSparkMax());
        aprilTagVision =
            new AprilTagVision(
                new AprilTagVisionIOLimelight("limelight", poseManager), poseManager);
        shooter =
            new Shooter(
                new Flywheels(new FlywheelsIOSparkMax()),
                new Pivot(new PivotIOSparkMax(), poseManager),
                new BeamBreakIORev(),
                new Feeder(new FeederIOSparkMax()));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                poseManager,
                driveCommandsConfig);
        intake = new Intake(new IntakeIOSim());
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {}, poseManager);
        shooter =
            new Shooter(
                new Flywheels(new FlywheelsIOSim()),
                new Pivot(new PivotIOSim(), poseManager),
                new BeamBreakIOSim(),
                new Feeder(new FeederIOSim()));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                poseManager,
                driveCommandsConfig);
        intake = new Intake(new IntakeIO() {});
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {}, poseManager);
        shooter =
            new Shooter(
                new Flywheels(new FlywheelsIO() {}),
                new Pivot(new PivotIO() {}, poseManager),
                new BeamBreakIO() {},
                new Feeder(new FeederIO() {}));
        break;
    }

    // Set up auto routines
    autoChooser.addDefaultOption("nothing", () -> {});
    autoChooser.addOption("source43", source43());
    autoChooser.addOption("centerCBA1", centerCBA1());
    // autoChooser.addOption("sourceCBA", sourceCBA());

    // Set up test routines
    if (!DriverStation.isFMSAttached()) {
      // Set up SysId routines
      autoChooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          driveSysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          driveSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption(
          "Drive SysId (Dynamic Forward)", driveSysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Dynamic Reverse)", driveSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    // Configure the button bindings
    configureButtonBindings();

    // Alerts for constants
    if (Constants.tuningMode) {
      new Alert("Tuning mode enabled", AlertType.INFO).set(true);
    }

    SmartDashboard.putData(intake.fullIntakeCmd().withTimeout(5).withName("Lower and Run Intake"));
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // Default cmds
    drive.setDefaultCommand(drive.joystickDrive());
    intake.setDefaultCommand(intake.raiseAndStopCmd());

    // Driver controls
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driver
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        poseManager.setPose(
                            new Pose2d(poseManager.getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    driver.leftBumper().onTrue(Commands.runOnce(() -> slowMode = !slowMode, drive));
    driver.b().whileTrue(drive.pointAtSpeakerCmd());
    driver
        .y()
        .whileTrue(drive.fullAutoDrive(() -> new Pose2d(1.815, 7.8, new Rotation2d(-Math.PI / 2))));

    // Operator controls
    operator
        .triangle()
        .whileTrue(intake.poopCmd().alongWith(shooter.setOuttaking()).withName("poop"));
    operator
        .square()
        .whileTrue(
            shooter
                .setIntaking(intake.intakeWorking)
                .deadlineWith(intake.intakeCmd(operator.cross()))
                .withName("setIntaking"));
    operator.circle().whileTrue(shootCmd());

    operator.povUp().onTrue(shooter.stopFlywheels());
    operator
        .povDown()
        .whileTrue(
            Commands.startEnd(
                () -> shooter.setSimNoteInShooter(false), () -> shooter.setSimNoteInShooter(true)));

    operator.L1().onTrue(shooter.setAmpShot());
    operator.R1().onTrue(shooter.setAutoAimShot());
    operator.L2().onTrue(shooter.setFeeding());
    operator.R2().onTrue(shooter.setManualSpeakerShot());
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

  private Command shootCmd() {
    return shooter.feedNoteToFlywheels().deadlineWith(intake.shootCmd()).withName("shootNote");
  }

  // Auto stuff
  public void setAutoIfChanged() {
    if (autoChooser.hasChanged(hashCode())) {
      autoChooser.get().run();
    }

    Logger.recordOutput("Drive/Choreo/intakingIndex", intakingIndex);
    Logger.recordOutput("Drive/Choreo/shootingIndex", shootingIndex);
  }

  private Trigger autoTrigger(BooleanSupplier condition) {
    return new Trigger(condition).and(DriverStation::isAutonomousEnabled);
  }

  private Trigger atStartOfAuto(Command command) {
    return new Trigger(DriverStation::isAutonomousEnabled).onTrue(command);
  }

  private Trigger resetPoseAtStart(ChoreoTrajectory firstTraj) {
    return atStartOfAuto(
        Commands.runOnce(
            () -> poseManager.setPose(AllianceFlipUtil.apply(firstTraj.getInitialPose()))));
  }

  private BooleanSupplier nearEndOf(Supplier<ChoreoTrajectory> traj, double tolerance) {
    return () ->
        poseManager.near(AllianceFlipUtil.apply(traj.get().getFinalPose().getTranslation()), .1);
  }

  private int intakingIndex;
  private int shootingIndex;
  private int pathIndex;

  private ChoreoTrajectory[] trajs;

  private Supplier<ChoreoTrajectory> intakingTraj() {
    return () -> trajs[intakingIndex];
  }

  private Supplier<ChoreoTrajectory> shootingTraj() {
    return () -> trajs[shootingIndex];
  }

  private Supplier<ChoreoTrajectory> pathTraj() {
    return () -> trajs[pathIndex];
  }

  private Command intakingTrajCmd() {
    return drive.runChoreoTrajSupplied(intakingTraj());
  }

  private Command shootingTrajCmd() {
    return drive.runChoreoTrajSupplied(shootingTraj());
  }

  private Command moveToNextIntakingPath() {
    return Commands.runOnce(
        () -> {
          if (trajs.length - 3 >= intakingIndex) intakingIndex += 2;
        });
  }

  private Command moveToNextShootingPath() {
    return Commands.runOnce(
        () -> {
          if (trajs.length - 3 >= shootingIndex) shootingIndex += 2;
        });
  }

  private Runnable source43() {
    return () -> {
      trajs =
          new ChoreoTrajectory[] {
            Choreo.getTrajectory("SourceTo4"),
            Choreo.getTrajectory("4ToShoot"),
            Choreo.getTrajectory("ShootTo3"),
            Choreo.getTrajectory("3ToShoot")
          };
      intakingIndex = 0;
      shootingIndex = 1;

      resetPoseAtStart(trajs[0]);
      atStartOfAuto(
          shooter
              .setManualSpeakerShot()
              .until(shooter::atDesiredAngle)
              .andThen(shootCmd())
              .withName("first shot"));
      autoTrigger(shooter::noteInShooter)
          .onFalse(
              Commands.runOnce(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(
                              intakingTrajCmd()
                                  .andThen(shootingTrajCmd())
                                  .andThen(drive.pointAtSpeakerCmd())
                                  .withName("followTrajsThenAim"))));
      autoTrigger(nearEndOf(intakingTraj(), 1))
          .onTrue(
              shooter
                  .setIntaking(intake.intakeWorking)
                  .deadlineWith(intake.fullIntakeCmd())
                  .andThen(moveToNextIntakingPath())
                  .withName("nearIntakePose"));
      autoTrigger(nearEndOf(shootingTraj(), .5))
          .and(shooter::noteInShooter)
          .onTrue(
              shooter
                  .setAutoAimShot()
                  .deadlineWith(moveToNextShootingPath())
                  .withName("setAutoAim"));
      autoTrigger(drive::thetaAtGoal).and(shooter::atDesiredAngle).onTrue(shootCmd());
    };
  }

  private Runnable centerCBA1() {
    pathIndex = 0;
    intakingIndex = 5;
    shootingIndex = 6;
    return () -> {
      trajs =
          new ChoreoTrajectory[] {
            Choreo.getTrajectory("CenterToC"),
            Choreo.getTrajectory("CToB"),
            Choreo.getTrajectory("BToA"),
            Choreo.getTrajectory("ATo1"),
            Choreo.getTrajectory("1ToShoot")
          };
      intakingIndex = 0;
      shootingIndex = 1;

      resetPoseAtStart(trajs[0]);
      atStartOfAuto(
          shooter
              .setManualSpeakerShot()
              .until(shooter::atDesiredAngle)
              .andThen(shootCmd())
              .withName("first shot"));
      autoTrigger(shooter::noteInShooter)
          .onFalse(
              Commands.runOnce(
                  () ->
                      CommandScheduler.getInstance()
                          .schedule(
                              intakingTrajCmd()
                                  .andThen(shootingTrajCmd())
                                  .andThen(drive.pointAtSpeakerCmd())
                                  .withName("followTrajsThenAim"))));
      autoTrigger(nearEndOf(pathTraj(), 1))
          .onTrue(
            //i know this is kinda spagetti but ill fix it after avi restructures again
              shooter
                  .setIntaking(intake.intakeWorking)
                  .deadlineWith(intake.fullIntakeCmd())
                  .until(shooter::noteInShooter)
                  .andThen(shooter.setAutoAimShot()));
        autoTrigger(drive::thetaAtGoal).and(shooter::atDesiredAngle)
        .onTrue(shootCmd());
                  
    };
  }

  private Runnable driveSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return () -> {
      atStartOfAuto(drive.sysIdQuasistatic(direction));
    };
  }

  private Runnable driveSysIdDynamic(SysIdRoutine.Direction direction) {
    return () -> {
      atStartOfAuto(drive.sysIdDynamic(direction));
    };
  }
}
