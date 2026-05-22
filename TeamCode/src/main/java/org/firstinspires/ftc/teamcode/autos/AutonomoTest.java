package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.commands.FireSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.ShootOnMoveCommand;
import org.firstinspires.ftc.teamcode.commands.TurretTrackCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "AUTO - Azul (Sem Gate)", group = "Competição")
public class AutonomoTest extends CommandOpMode {

    private Follower follower;

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private IndexerSubsystem indexer;
    private IntakeSubsystem intake;
    private HoodSubsystem hood;
    private VisionSubsystem vision;

    // =========================================================
    // POSES (todas com 180°)
    // =========================================================
    private final Pose startPose = new Pose(31.5, 133.5, Math.toRadians(180));

    private final Pose pose1 = new Pose(54.3, 85.0, Math.toRadians(180)); // Lançar
    private final Pose pose2 = new Pose(42.6, 85.0, Math.toRadians(180)); // Liga intake
    private final Pose pose3 = new Pose(18.8, 85.0, Math.toRadians(180)); // Coleta
    private final Pose pose4 = new Pose(50.7, 85.0, Math.toRadians(180)); // Lançar
    private final Pose pose5 = new Pose(43.9, 61.8, Math.toRadians(180)); // Liga intake
    private final Pose pose6 = new Pose(17.7, 61.8, Math.toRadians(180)); // Coleta
    private final Pose pose7 = new Pose(56.1, 81.6, Math.toRadians(180)); // Lançar
    private final Pose pose8 = new Pose(48.7, 70.1, Math.toRadians(180)); // Sair da área

    // RPM de lançamento no autônomo
    private static final double SHOOT_RPM = 3800;

    // Multiplicador do hood

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        drive   = new DriveSubsystem(hardwareMap, hardwareMap.voltageSensor.iterator().next(), follower, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        turret  = new TurretSubsystem(hardwareMap, telemetry);
        indexer = new IndexerSubsystem(hardwareMap);
        intake  = new IntakeSubsystem(hardwareMap);
        hood    = new HoodSubsystem(hardwareMap, telemetry);
        vision  = new VisionSubsystem(hardwareMap, telemetry);

        turret.setDefaultCommand(new TurretTrackCommand(
                turret, drive, vision, hood,
                () -> FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL),
                () -> FieldConstants.getTargetTagId(FieldConstants.TargetGoal.GOAL),
                () -> shooter.getCurrentRPM()
        ));

        shooter.stop();

        // =========================================================
        // SEQUÊNCIA PRINCIPAL DO AUTÔNOMO
        // =========================================================
        schedule(new SequentialCommandGroup(

                // --- PATH 1: Ir para posição de lançamento ---
                new InstantCommand(() -> {
                    followPath(pose1, false);
                    Pose2d alvo = FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL);
                    double distanciaEsperada = Math.hypot(alvo.getX() - pose1.getX(), alvo.getY() - pose1.getY());
                    shooter.setRPMFromDistance(distanciaEsperada);
                }),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(1500),
                new FireSequenceCommand(indexer,intake,hood),

                // --- LANÇAMENTO 1 ---

                new WaitCommand(200),

                // --- PATH 2: Liga intake e avança ---
                new InstantCommand(() -> {
                    intake.setPower(1.0);
                    followPath(pose2, false);
                }),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(200),

                // --- PATH 3: Coleta enquanto recua ---
                new InstantCommand(() -> followPath(pose3, false)),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(1000),
                new InstantCommand(() -> intake.stop()),

                // --- PATH 4: Ir para posição de lançamento ---
                new InstantCommand(() -> {
                    followPath(pose4, false);
                    Pose2d alvo = FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL);
                    double distanciaEsperada = Math.hypot(alvo.getX() - pose4.getX(), alvo.getY() - pose4.getY());
                    shooter.setRPMFromDistance(distanciaEsperada);
                }),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(500),

                // --- LANÇAMENTO 2 ---
                new FireSequenceCommand(indexer, intake, hood),
                new WaitCommand(200),

                // --- PATH 5: Liga intake e avança para nova área ---
                new InstantCommand(() -> {
                    intake.setPower(1.0);
                    followPath(pose5, false);
                }),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(500),

                // --- PATH 6: Coleta enquanto recua ---
                new InstantCommand(() -> followPath(pose6, false)),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(1000),
                new InstantCommand(() -> intake.stop()),

                // --- PATH 7: Ir para posição de lançamento ---
                new InstantCommand(() -> {
                    followPath(pose7, false);
                    Pose2d alvo = FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL);
                    double distanciaEsperada = 5+Math.hypot(alvo.getX() - pose7.getX(), alvo.getY() - pose7.getY());
                    shooter.setRPMFromDistance(distanciaEsperada);
                }),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(500),

                // --- LANÇAMENTO 3 ---
                new FireSequenceCommand(indexer, intake, hood),
                new WaitCommand(200),

                // --- PATH 8: Sair da área ---
                new InstantCommand(() -> {
                    shooter.stop();
                    followPath(pose8, true);
                }),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(1500),

                new InstantCommand(() -> turret.travarNoZero())

        ));
    }


    private void followPath(Pose target, boolean isLast) {
        PathChain chain = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), target))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), target.getHeading())
                .build();
        follower.followPath(chain, !isLast);
    }

    @Override
    public void run() {
        super.run();

        follower.update();
        telemetry.update();

        // Salva a posição continuamente para quando o TeleOp assumir
        PoseStorage.storePose(follower.getPose());
        PoseStorage.storeTurretAngle(turret.getCurrentAngle());
    }
}