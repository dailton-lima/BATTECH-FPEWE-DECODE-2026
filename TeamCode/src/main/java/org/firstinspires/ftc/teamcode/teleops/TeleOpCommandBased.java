package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.ShootOnMoveCommand;
import org.firstinspires.ftc.teamcode.commands.TurretTrackCommand;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

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

    @Override
    public void initialize() {
        // Inicialização de todos os subsistemas
        drive = new DriveSubsystem(hardwareMap, hardwareMap.voltageSensor.iterator().next());
        turret = new TurretSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        indexer = new IndexerSubsystem(hardwareMap);
        hood = new HoodSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);

        piloto1 = new GamepadEx(gamepad1);
        piloto2 = new GamepadEx(gamepad2);

        // =========================================================
        // PILOTO 1: DRIVE + INTAKE (O "MOTORISTA")
        // =========================================================

        // Controle Mecanum Field-Oriented (Sempre ativo)
        drive.setDefaultCommand(new RunCommand(() -> {
            drive.drive(
                    piloto1.getLeftX(),
                    -piloto1.getLeftY(),
                    piloto1.getRightX() * driveSpeed
            );
        }, drive));

        turret.setDefaultCommand(new TurretTrackCommand(
                turret, drive, vision, shooter, hood,
                () -> FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL),
                () -> FieldConstants.getTargetTagId(FieldConstants.TargetGoal.GOAL)
        ));

        piloto1.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> drive.resetHeading()));

        // SLOW MODE: Reduz velocidade para precisão
        piloto1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .toggleWhenPressed(
                        new InstantCommand(() -> driveSpeed = 0.4),
                        new InstantCommand(() -> driveSpeed = 1.0)
                );

        // INTAKE NOS GATILHOS (Analógico = Controle de Força)
        new RunCommand(() -> {
            double coletar = piloto1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            double expelir = piloto1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            if (coletar > 0.1) {
                intake.setPower(coletar);
            } else if (expelir > 0.1) {
                intake.setPower(-expelir);
            } else {
                intake.stop();
            }
        }, intake).schedule();


        // =========================================================
        // PILOTO 2: TURRET + SHOOTER + HOOD (O "ARTILHEIRO")
        // =========================================================

        // RB (Hold): ATIVA A MIRA AUTOMÁTICA E O DISPARO
        // O Piloto 2 segura este botão enquanto o Piloto 1 dirige livremente.
        // O robô vai mirar, ajustar o capô e atirar assim que estiver cravado.
        piloto2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new ShootOnMoveCommand(
                        turret, drive, vision, shooter, indexer, intake, hood,
                        () -> FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL),
                        () -> FieldConstants.getTargetTagId(FieldConstants.TargetGoal.GOAL)
                ));

        // LB (Toggle): Ligar o Shooter antecipadamente
        piloto2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(
                        new InstantCommand(() -> shooter.setTargetRPM(6000)),
                        new InstantCommand(() -> shooter.stop())
                );

        // PRESETS DE DISTÂNCIA
        piloto2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> shooter.setTargetRPM(6000))); // High Basket
        piloto2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> shooter.setTargetRPM(4000))); // Low Basket

        // AJUSTE FINO (D-Pad): Mover a Turret manualmente
        new RunCommand(() -> {
            if (gamepad2.dpad_left) turret.setAngle(turret.getCurrentAngle() - 1);
            if (gamepad2.dpad_right) turret.setAngle(turret.getCurrentAngle() + 1);
        }, turret).schedule();
    }
}