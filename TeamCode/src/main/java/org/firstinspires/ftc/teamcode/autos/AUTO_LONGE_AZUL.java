package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.commands.FireSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.TurretTrackCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "AUTO - AZUL LONGE", group = "Competição")
public class AUTO_LONGE_AZUL extends CommandOpMode {

    private Follower follower;

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private IndexerSubsystem indexer;
    private IntakeSubsystem intake;
    private HoodSubsystem hood;
    private VisionSubsystem vision;

    // =========================================================
    // POSES SEQUENCIAIS (Destinos de cada Path)
    // =========================================================
    private final Pose startPose = new Pose(55.5, 7.8, Math.toRadians(180));

    // Ações mapeadas para as 6 coordenadas do seu buildPaths
    private final Pose pose1 = new Pose(56.0, 36.0, Math.toRadians(180)); // Lançar 1
    private final Pose pose2 = new Pose(16.0, 36.0, Math.toRadians(180)); // Desce para coletar
    private final Pose pose3 = new Pose(56.0, 11.0, Math.toRadians(180)); // Recua e Lança 2
    private final Pose pose4 = new Pose(12.0, 11.0, Math.toRadians(180)); // Vai ao fundo coletar
    private final Pose pose5 = new Pose(12.5, 11.0, Math.toRadians(190)); // Volta e Lança 3
    private final Pose pose6 = new Pose(12.5, 12.0, Math.toRadians(190)); // Ajuste fino / Estacionar
    private final Pose pose7 = new Pose(12.5, 13.0, Math.toRadians(170)); // Volta e Lança 3
    private final Pose pose8 = new Pose(12.5, 11.0, Math.toRadians(180)); // Ajuste fino / Estacionar
    private final Pose pose9 = new Pose(56.0, 11.0, Math.toRadians(180)); // Volta e Lança 3
    private final Pose pose10 = new Pose(12.5, 11.0, Math.toRadians(180)); // Ajuste fino / Estacionar
    private final Pose pose11 = new Pose(56.0, 11.0, Math.toRadians(180)); // Volta e Lança 3
    private final Pose pose12 = new Pose(36.0, 11.0, Math.toRadians(180)); // Ajuste fino / Estacionar

    // =========================================================
    // VARIÁVEIS DE TRAJETÓRIA (PathChains)
    // =========================================================
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // 1. CONSTRÓI TODOS OS CAMINHOS PRIMEIRO
        buildPaths();

        drive   = new DriveSubsystem(hardwareMap, hardwareMap.voltageSensor.iterator().next(), follower, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        turret  = new TurretSubsystem(hardwareMap, telemetry);
        indexer = new IndexerSubsystem(hardwareMap);
        intake  = new IntakeSubsystem(hardwareMap);
        hood    = new HoodSubsystem(hardwareMap, telemetry);
        vision  = new VisionSubsystem(hardwareMap, telemetry);



        shooter.stop();

        // =========================================================
        // SEQUÊNCIA PRINCIPAL DO AUTÔNOMO
        // =========================================================
        schedule(new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new TurretTrackCommand(
                                turret, drive, vision, hood,
                                () -> FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL),
                                () -> FieldConstants.getTargetTagId(FieldConstants.TargetGoal.GOAL),
                                () -> shooter.getCurrentRPM()
                        ),
                        new SequentialCommandGroup(
                                // --- Lançar ---
                                new InstantCommand(() -> {
                                    shooter.setTargetRPM(4550);
                                }),
                                new WaitUntilCommand(() -> shooter.getCurrentRPM()>4300),
                                new WaitCommand(200),
                                new FireSequenceCommand(indexer, intake, hood),

                                // --- PATH 1: Ir para posição de coleta ---
                                new InstantCommand(() -> {
                                    follower.followPath(path1, false);
                                }),
                                new WaitUntilCommand(() -> !follower.isBusy()),

                                // --- PATH 2: Coletar ---
                                new InstantCommand(() -> {
                                    follower.followPath(path2, false);
                                    intake.setPower(1.0);
                                }),
                                new WaitUntilCommand(() -> !follower.isBusy()),
                                new WaitCommand(300),
                                new InstantCommand(() -> intake.stop()),

                                // --- PATH 3: Ir para posição de lançamento ---
                                new InstantCommand(() -> {
                                    follower.followPath(path3, false);
                                    shooter.setTargetRPM(4550);
                                }),
                                new WaitUntilCommand(() -> !follower.isBusy()),
                                new WaitUntilCommand(() -> shooter.getCurrentRPM()>4300),
                                new WaitCommand(200),
                                new FireSequenceCommand(indexer, intake, hood),

                                // --- PATH 4: Vai fundo para o Gate (Coleta) ---
                                new InstantCommand(() -> {
                                    follower.followPath(path4, false);

                                }),
                                new WaitCommand(400),
                                new InstantCommand(() -> intake.setPower(1.0)),
                                new WaitUntilCommand(() -> !follower.isBusy()),

                                // --- PATH 5: Vai fundo para o Gate (Coleta) ---
                                new InstantCommand(() -> {
                                    follower.followPath(path5, false);

                                }),
                                new WaitUntilCommand(() -> !follower.isBusy()),

                                // --- PATH 6: Vai fundo para o Gate (Coleta) ---
                                new InstantCommand(() -> {
                                    follower.followPath(path6, false);

                                }),
                                new WaitUntilCommand(() -> !follower.isBusy()),

                                // --- PATH 7: Vai fundo para o Gate (Coleta) ---
                                new InstantCommand(() -> {
                                    follower.followPath(path7, false);

                                }),
                                new WaitUntilCommand(() -> !follower.isBusy()),

                                // --- PATH 8: Vai fundo para o Gate (Coleta) ---
                                new InstantCommand(() -> {
                                    follower.followPath(path8, false);

                                }),
                                new WaitUntilCommand(() -> !follower.isBusy()),
                                new WaitCommand(200),
                                new InstantCommand(() -> intake.stop()),

                                // --- PATH 9: Retorna do Gate para Lançamento 3 ---
                                new InstantCommand(() -> {
                                    follower.followPath(path9, false);
                                    shooter.setTargetRPM(4550);
                                }),
                                new WaitUntilCommand(() -> !follower.isBusy()),
                                new WaitUntilCommand(() -> shooter.getCurrentRPM()>4300),
                                new WaitCommand(200),
                                new FireSequenceCommand(indexer, intake, hood),

                                // --- PATH 9: Retorna do Gate para Lançamento 3 ---
                                new InstantCommand(() -> {
                                    follower.followPath(path10, false);
                                    intake.setPower(1.0);
                                }),
                                new WaitUntilCommand(() -> !follower.isBusy()),
                                new WaitCommand(300),
                                new InstantCommand(() -> intake.stop()),

                                // --- PATH 3: Ir para posição de lançamento ---
                                new InstantCommand(() -> {
                                    follower.followPath(path11, false);
                                    shooter.setTargetRPM(4550);
                                }),
                                new WaitUntilCommand(() -> !follower.isBusy()),
                                new WaitUntilCommand(() -> shooter.getCurrentRPM()>4300),
                                new WaitCommand(200),
                                new FireSequenceCommand(indexer, intake, hood),

                                // --- PATH 10: Ajuste Fino / Estacionar ---
                                new InstantCommand(() -> {
                                    shooter.stop();
                                    follower.followPath(path12, true);
                                }),
                                new WaitUntilCommand(() -> !follower.isBusy()),

                                // Trava a torre no centro para a transição do TeleOp
                                new InstantCommand(() -> turret.travarNoZero())
                        )
                )
                )
        );
    }

    /**
     * Constrói as trajetórias amarradas às variáveis sequenciais
     */
    private void buildPaths() {
        // Path 1: Da posição inicial até a primeira pose de disparo
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), pose1.getHeading())
                .build();

        // Path 2: Desce para ligar o intake
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();

        // Path 3: Recua coletando
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
                .build();

        // Path 4: Vai lá no fundo coletar
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .build();

        // Path 5: Volta para a zona de lançamento
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(pose4, pose5))
                .setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading())
                .build();

        // Path 6: Ajuste fino de coleta / Estacionar
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(pose5, pose6))
                .setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading())
                .build();

        // Path 7: Ajuste fino de coleta / Estacionar
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(pose6, pose7))
                .setLinearHeadingInterpolation(pose6.getHeading(), pose7.getHeading())
                .build();

        // Path 8: Ajuste fino de coleta / Estacionar
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(pose7, pose8))
                .setLinearHeadingInterpolation(pose7.getHeading(), pose8.getHeading())
                .build();

        // Path 9: Ajuste fino de coleta / Estacionar
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(pose8, pose9))
                .setLinearHeadingInterpolation(pose8.getHeading(), pose9.getHeading())
                .build();

        // Path 6: Ajuste fino de coleta / Estacionar
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(pose9, pose10))
                .setLinearHeadingInterpolation(pose9.getHeading(), pose10.getHeading())
                .build();

        // Path 9: Ajuste fino de coleta / Estacionar
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(pose10, pose11))
                .setLinearHeadingInterpolation(pose10.getHeading(), pose11.getHeading())
                .build();

        // Path 6: Ajuste fino de coleta / Estacionar
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(pose11, pose12))
                .setLinearHeadingInterpolation(pose11.getHeading(), pose12.getHeading())
                .build();
    }

    @Override
    public void run() {
        super.run();

        follower.update();
        telemetry.update();

        PoseStorage.storePose(follower.getPose());
        PoseStorage.storeTurretAngle(turret.getCurrentAngle());
    }
}