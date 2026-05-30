package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.FireSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.ShootOnMoveCommand;
import org.firstinspires.ftc.teamcode.commands.TurretTrackCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp(name = "TELEOPCOMMAND", group = "Competição")
public class TeleOpCommandBased extends CommandOpMode {

    private DriveSubsystem drive;
    private TurretSubsystem turret;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;
    private HoodSubsystem hood;
    private VisionSubsystem vision;

    private GamepadEx piloto1;
    private GamepadEx piloto2;

    private double driveSpeed = 1.0;

    private Follower follower;

    private Pose startPoseReset;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);

        // =========================================================
        // POSE INICIAL
        // =========================================================
        if (PoseStorage.getPose().getX() > 0  && PoseStorage.getPose().getY() > 0) {
            follower.setStartingPose(PoseStorage.getPose());
        } else {
            follower.setStartingPose(new Pose(134, 8, Math.toRadians(90)));
        }

        if (FieldConstants.activeAlliance == FieldConstants.Alliance.RED) {
            startPoseReset = new Pose(8, 8, Math.toRadians(90));
        } else {
            startPoseReset = new Pose(134, 8, Math.toRadians(90));
        }

        double poseTurret = PoseStorage.getTurretAngle();

        drive = new DriveSubsystem(hardwareMap, hardwareMap.voltageSensor.iterator().next(), follower, telemetry);
        turret = new TurretSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap);
        indexer = new IndexerSubsystem(hardwareMap);
        hood = new HoodSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap, telemetry);

        piloto1 = new GamepadEx(gamepad1);
        piloto2 = new GamepadEx(gamepad2);

        follower.startTeleOpDrive();

        // =========================================================
        // PILOTO 1: DRIVE + INTAKE
        // =========================================================

        piloto1.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> follower.setPose(startPoseReset)));

        drive.setDefaultCommand(new RunCommand(() -> {
            drive.drive(
                    -piloto1.getLeftX(),
                    piloto1.getLeftY(),
                    -piloto1.getRightX() * driveSpeed
            );
        }, drive));

        // Comando padrão da Torreta continua a ser o Track Automático
        turret.setDefaultCommand(new TurretTrackCommand(
                turret, drive, vision, hood,
                () -> FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL),
                () -> FieldConstants.getTargetTagId(FieldConstants.TargetGoal.GOAL),
                () -> shooter.getCurrentRPM()
        ));

        piloto1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .toggleWhenPressed(
                        new InstantCommand(() -> driveSpeed = 0.4),
                        new InstantCommand(() -> driveSpeed = 1.0)
                );

        intake.setDefaultCommand(new RunCommand(() -> {
            double coletar = piloto1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            double expelir = piloto1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            if (coletar > 0.1) {
                intake.setPower(coletar);
            } else if (expelir > 0.1) {
                intake.setPower(-expelir);
            } else {
                intake.stop();
            }
        }, intake));

        // =========================================================
        // PILOTO 2: TURRET + SHOOTER + HOOD + CALIBRAÇÃO MANUAL
        // =========================================================

        piloto2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new ShootOnMoveCommand(
                        turret, drive, vision, shooter, indexer, intake, hood,
                        () -> FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL),
                        () -> FieldConstants.getTargetTagId(FieldConstants.TargetGoal.GOAL),
                        telemetry, false, true
                ));

        // ---------------------------------------------------------
        // FAIL-SAFE DA TORRETA (Calibração em Jogo)
        // ---------------------------------------------------------
        // Botão X: Desliga o PID e entra no modo de calibração manual
        piloto2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    turret.isManualMode = true;
                }));

        // CORREÇÃO: Removido o requerimento da "turret" para não matar o TurretTrackCommand!
        new RunCommand(() -> {
            if (turret.isManualMode) {
                // Lê o analógico direito (eixo X)
                double turnPower = piloto2.getRightX();
                turret.setManualPower(turnPower * 0.2); // Força reduzida para maior precisão visual
            }
        }).schedule(); // <-- Repare que não há ", turret" aqui dentro do parênteses.

        // Botão B: Zera o Encoder na posição atual e volta para o Modo Automático
        piloto2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    if (turret.isManualMode) {
                        turret.resetEncoder();
                        turret.isManualMode = false;
                    }
                }));
        // ---------------------------------------------------------

        piloto2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> indexer.unlock()));
        piloto2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> indexer.lock()));

        piloto2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> hood.setPosition(hood.getServoPosition() + 0.1)));
        piloto2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> hood.setPosition(hood.getServoPosition() - 0.1)));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("POSE INICIAL", PoseStorage.currentPose);
        telemetry.update();
    }
}
