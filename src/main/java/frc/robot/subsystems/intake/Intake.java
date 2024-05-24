package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

import frc.robot.util.LoggedTunableNumber;

public class Intake {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/Gains/kP", gains.kP());

    private static final LoggedTunableNumber loweredAngle = new LoggedTunableNumber("Intake/Angles/lowered", kLoweredAngle);
    private static final LoggedTunableNumber raisedAngle = new LoggedTunableNumber("Intake/Angles/raised", kRaisedAngle);
    private static final LoggedTunableNumber climbingAngle = new LoggedTunableNumber("Intake/Angles/climbing", kClimbingAngle);

    private static final LoggedTunableNumber rollersSpeed = new LoggedTunableNumber("Intake/Speeds/intakeRollers", kRollersSpeed);
    private static final LoggedTunableNumber indexerSpeed = new LoggedTunableNumber("Intake/Speeds/indexer", kIndexerSpeed);

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;

        io.setPID(kP.get());
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Update controllers
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> io.setPID(kP.get()), kP);
    }
}
