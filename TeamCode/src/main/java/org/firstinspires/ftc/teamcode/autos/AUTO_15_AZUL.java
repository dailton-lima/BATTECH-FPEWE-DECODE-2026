package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "AUTO - Azul 15 GATE", group = "Competição AZUL")
public class AUTO_15_AZUL extends CommandOpMode {

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
    private final Pose startPose = new Pose(31.5, 133.5, Math.toRadians(180));

    private final Pose pose1  = new Pose(54.3, 85.0, Math.toRadians(180)); // Lançar 1
    private final Pose pose2  = new Pose(42.5, 61.0, Math.toRadians(180)); // Desce para coleta
    private final Pose pose3  = new Pose(15.0, 61.0, Math.toRadians(180)); // Coleta 1
    private final Pose pose4  = new Pose(54.0, 82.0, Math.toRadians(180)); // Lançar 2
    private final Pose pose5  = new Pose(16.2, 62.0, Math.toRadians(160)); // Ida ao Gate
    private final Pose pose6  = new Pose(9.5,  60.0, Math.toRadians(160)); // Coleta fina Gate

    // Ajuste milimétrico de início do Path 7
    private final Pose poseAjuste7 = new Pose(9.2, 60.0, Math.toRadians(160));
    private final Pose pose7  = new Pose(54.0, 82.0, Math.toRadians(180)); // Lançar 3

    private final Pose pose8  = new Pose(16.2, 62.0, Math.toRadians(160)); // Volta ao Gate
    private final Pose pose9  = new Pose(9.5,  60.0, Math.toRadians(160)); // Coleta fina Gate 2

    // Ajuste milimétrico de início do Path 10
    private final Pose poseAjuste10 = new Pose(9.2, 60.0, Math.toRadians(160));
    private final Pose pose10 = new Pose(54.0, 84.0, Math.toRadians(180)); // Lançar 4

    private final Pose pose11 = new Pose(16.0, 85.0, Math.toRadians(180)); // Coleta Longe

    // Ajuste milimétrico de início do Path 12
    private final Pose poseAjuste12 = new Pose(16.0, 84.0, Math.toRadians(180));
    private final Pose pose12 = new Pose(54.0, 85.0, Math.toRadians(180)); // Lançar 5

    private final Pose pose13 = new Pose(18.0, 85.0, Math.toRadians(180)); // Estacionar

    // Pontos de Controle (Curvas de Bézier)
    private final Pose control1 = new Pose(38.0, 68.0);
    private final Pose control2 = new Pose(35.0, 62.0);
    private final Pose control3 = new Pose(42.0, 62.0);

    // =========================================================
    // VARIÁVEIS DE TRAJETÓRIA (PathChains)
    // =========================================================
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12, path13;

    private long pathStartTime = 0;

    @Override
    public void initialize() {
        FieldConstants.activeAlliance = FieldConstants.Alliance.BLUE;

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

                // --- PATH 1: Ir para posição de lançamento ---
                new InstantCommand(() -> {
                    follower.followPath(path1, false);
                    Pose2d alvo = FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL);
                    double distanciaEsperada = Math.hypot(alvo.getX() - pose1.getX(), alvo.getY() - pose1.getY());
                    shooter.setRPMFromDistance(distanciaEsperada);
                }),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitUntilCommand(() -> shooter.getCurrentRPM()>3000),
                new FireSequenceCommand(indexer,intake,hood),

                // --- PATH 2: Liga intake e avança ---
                new InstantCommand(() -> follower.followPath(path2, false)),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new InstantCommand(() -> intake.setPower(1.0)),

                // --- PATH 3: Coleta enquanto recua ---
                new InstantCommand(() -> follower.followPath(path3, false)),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(200),
                new InstantCommand(() -> intake.stop()),

                // --- PATH 4 (CURVA): Ir para posição de lançamento ---
                new InstantCommand(() -> {
                    follower.followPath(path4, false);
                    Pose2d alvo = FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL);
                    double distanciaEsperada = Math.hypot(alvo.getX() - pose4.getX(), alvo.getY() - pose4.getY());
                    shooter.setRPMFromDistance(distanciaEsperada);
                }),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new FireSequenceCommand(indexer, intake, hood),

                // --- PATH 5 (CURVA): Abre o gate-
                new InstantCommand(() -> {
                    pathStartTime = System.currentTimeMillis();
                    follower.followPath(path5, false);
    // ... os seus comandos de shooter ...
                }),
                new WaitUntilCommand(() -> !follower.isBusy() || (System.currentTimeMillis() - pathStartTime > 2500)),
                new WaitCommand(400),

                // --- PATH 6: Coleta no gate ---
                new InstantCommand(() -> {
                    follower.followPath(path6, false);
                    intake.setPower(1.0);
                }),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(1000),
                new InstantCommand(() -> intake.stop()),

                // --- PATH 7 (CURVA): Ir para posição de lançamento ---
                new InstantCommand(() -> {
                    follower.followPath(path7, false);
                    Pose2d alvo = FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL);
                    double distanciaEsperada =  Math.hypot(alvo.getX() - pose7.getX(), alvo.getY() - pose7.getY());
                    shooter.setRPMFromDistance(distanciaEsperada);
                }),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new FireSequenceCommand(indexer, intake, hood),

                // --- PATH 8 (CURVA): Abre o gate ---
                new InstantCommand(() -> {
                    pathStartTime = System.currentTimeMillis();
                    follower.followPath(path8, false);
    // ... os seus comandos de shooter ...
                }),
                new WaitUntilCommand(() -> !follower.isBusy() || (System.currentTimeMillis() - pathStartTime > 2500)),
                new WaitCommand(400),

                // --- PATH 9: Coleta no gate ---
                new InstantCommand(() -> {
                    follower.followPath(path9, false);
                    intake.setPower(1.0);
                }),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new WaitCommand(1000),
                new InstantCommand(() -> intake.stop()),

                // --- PATH 10: Ir para posição de lançamento ---
                new InstantCommand(() -> {
                    follower.followPath(path10, false);
                    Pose2d alvo = FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL);
                    double distanciaEsperada = 5 + Math.hypot(alvo.getX() - pose10.getX(), alvo.getY() - pose10.getY());
                    shooter.setRPMFromDistance(distanciaEsperada);
                }),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new FireSequenceCommand(indexer, intake, hood),

                // --- PATH 11: Liga intake e avança ---
              //  new InstantCommand(() -> {
              //      follower.followPath(path11, false);
              //      intake.setPower(1.0);
              //  }),
              //  new WaitUntilCommand(() -> !follower.isBusy()),
              //  new InstantCommand(() -> intake.stop()),

                // --- PATH 12 (CURVA): Ir para posição de lançamento ---
               // new InstantCommand(() -> {
               //     follower.followPath(path12, false);
               //     Pose2d alvo = FieldConstants.getTargetPose(FieldConstants.TargetGoal.GOAL);
               //     double distanciaEsperada = Math.hypot(alvo.getX() - pose12.getX(), alvo.getY() - pose12.getY());
               //     shooter.setRPMFromDistance(distanciaEsperada);
              //  }),
             //   new WaitUntilCommand(() -> !follower.isBusy()),
              //  new FireSequenceCommand(indexer, intake, hood),

                // --- PATH 13: Estacionar ---
                new InstantCommand(() -> {
                    follower.followPath(path13, false);
                    turret.travarNoZero();
                }),
                new WaitUntilCommand(() -> !follower.isBusy())

                )
                )
                )
        );
    }

    /**
     * Constrói as trajetórias com curvas e ângulos dinâmicos vinculados às variáveis sequenciais.
     */
    private void buildPaths() {
        // Path 1
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), pose1.getHeading())
                .build();

        // Path 2
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();

        // Path 3
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
                .build();

        // Path 4 (CURVA)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(pose3, control1, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .build();

        // Path 5 (CURVA)
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(pose4, control2, pose5))
                .setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading())
                .build();

        // Path 6
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(pose5, pose6))
                .setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading())
                .build();

        // Path 7 (CURVA) - Inicia do poseAjuste7
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(poseAjuste7, control3, pose7))
                .setLinearHeadingInterpolation(poseAjuste7.getHeading(), pose7.getHeading())
                .build();

        // Path 8 (CURVA)
        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(pose7, control2, pose8))
                .setLinearHeadingInterpolation(pose7.getHeading(), pose8.getHeading())
                .build();

        // Path 9
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(pose8, pose9))
                .setLinearHeadingInterpolation(pose8.getHeading(), pose9.getHeading())
                .build();

        // Path 10 (CURVA) - Inicia do poseAjuste10
        path10 = follower.pathBuilder()
                .addPath(new BezierCurve(poseAjuste10, control3, pose10))
                .setLinearHeadingInterpolation(poseAjuste10.getHeading(), pose10.getHeading())
                .build();

        // Path 11
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(pose10, pose11))
                .setLinearHeadingInterpolation(pose10.getHeading(), pose11.getHeading())
                .build();

        // Path 12 - Inicia do poseAjuste12
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(poseAjuste12, pose12))
                .setLinearHeadingInterpolation(poseAjuste12.getHeading(), pose12.getHeading())
                .build();

        // Path 13
        path13 = follower.pathBuilder()
                .addPath(new BezierLine(pose12, pose13))
                .setLinearHeadingInterpolation(pose12.getHeading(), pose13.getHeading())
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
