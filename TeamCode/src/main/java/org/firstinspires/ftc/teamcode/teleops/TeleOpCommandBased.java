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

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);

        // =========================================================
        // POSE INICIAL: pega do autônomo se existir, senão usa padrão
        // =========================================================
        if (PoseStorage.getPose().getX() > 0  && PoseStorage.getPose().getY() > 0) {
            follower.setStartingPose(PoseStorage.getPose());
        } else {
            follower.setStartingPose(new Pose(134, 8, Math.toRadians(90)));
        }

        double poseTurret = PoseStorage.getTurretAngle();

        drive = new DriveSubsystem(hardwareMap, hardwareMap.voltageSensor.iterator().next(), follower, telemetry);
        turret = new TurretSubsystem(hardwareMap, telemetry);
        turret.loadStartingAngle(poseTurret);

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

        drive.setDefaultCommand(new RunCommand(() -> {
            drive.drive(
                    -piloto1.getLeftX(),
                    piloto1.getLeftY(),
                    -piloto1.getRightX() * driveSpeed
            );
        }, drive));

        turret.setDefaultCommand(new TurretTrackCommand(
                turret, drive, vision, shooter, hood,
                () -> FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL),
                () -> FieldConstants.getTargetTagId(FieldConstants.TargetGoal.GOAL)
        ));

        piloto1.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> drive.resetHeading()));

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
        // PILOTO 2: TURRET + SHOOTER + HOOD
        // =========================================================

        piloto2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new ShootOnMoveCommand(
                        turret, drive, vision, shooter, indexer, intake, hood,
                        () -> FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL),
                        () -> FieldConstants.getTargetTagId(FieldConstants.TargetGoal.GOAL),
                        telemetry
                ))
                .whenReleased(new InstantCommand(() -> shooter.stop()));

        piloto2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(
                        new InstantCommand(() -> shooter.setTargetRPM(6000)),
                        new InstantCommand(() -> shooter.stop())
                );

        piloto2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> shooter.setTargetRPM(4000)));
        piloto2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> shooter.setTargetRPM(3500)));

        piloto2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new FireSequenceCommand(indexer, intake, hood));

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