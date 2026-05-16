package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.ShotSolution;

import java.util.function.Supplier;

public class ShootOnMoveCommand extends CommandBase {

    private final TurretSubsystem turret;
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;
    private final HoodSubsystem hood;

    private final Supplier<Pose2d> targetPoseSupplier;
    private final Supplier<Integer> targetTagIdSupplier;

    private final Telemetry telemetry;


    private long lastFireTime = 0;
    private static final long FIRE_COOLDOWN_MS = 500;

    public ShootOnMoveCommand(TurretSubsystem turret, DriveSubsystem drive, VisionSubsystem vision,
                              ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake, HoodSubsystem hood,
                              Supplier<Pose2d> targetPoseSupplier, Supplier<Integer> targetTagIdSupplier, Telemetry telemetry) {
        this.turret = turret;
        this.drive = drive;
        this.vision = vision;
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.hood = hood;
        this.targetPoseSupplier = targetPoseSupplier;
        this.targetTagIdSupplier = targetTagIdSupplier;
        this.telemetry = telemetry;

        addRequirements(turret, shooter, hood);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d alvoAtual = targetPoseSupplier.get();
        int tagAtiva = targetTagIdSupplier.get();

        Pose2d poseAtual = drive.getPose();
        Pose2d velAtual = drive.getVelocity();
        double headingRad = poseAtual.getHeading();
        double headingGraus = Math.toDegrees(headingRad);

        // ====================================================================
        // NOVO: OFFSET FÍSICO DA TORRE (Translação Cinemática)
        // ====================================================================
        // COLOQUE AQUI AS MEDIDAS REAIS COM A FITA MÉTRICA (em polegadas)
        // Ex: Se o eixo da torre está 5 polegadas para trás do centro do robô -> X = -5.0
        double OFFSET_X = -2.0;
        double OFFSET_Y = 0.0;  // 0 se a torre estiver bem no meio do robô na lateral

        // Calcula onde a torre realmente está no campo naquele momento
        double torreX = poseAtual.getX() + (OFFSET_X * Math.cos(headingRad) - OFFSET_Y * Math.sin(headingRad));
        double torreY = poseAtual.getY() + (OFFSET_X * Math.sin(headingRad) + OFFSET_Y * Math.cos(headingRad));

        // ====================================================================
        // 1. DISTÂNCIA 100% ODOMETRIA (Usando as coordenadas da Torre!)
        // ====================================================================
        // Note que mudamos 'poseAtual' para 'torreX' e 'torreY'
        double distOdo = Math.hypot(alvoAtual.getX() - torreX, alvoAtual.getY() - torreY);

        double distFinal = distOdo;

        telemetry.addData("MIRA - Distancia (Odo Torre - Usada)", distFinal);
        if (vision.hasTarget(tagAtiva)) {
            telemetry.addData("MIRA - Distancia (Camera - Ignorada)", 16 + vision.getDistance(tagAtiva));
        }

        // ====================================================================
        // 2. MATEMÁTICA DA TORRE (Usando as coordenadas da Torre!)
        // ====================================================================
        double anguloGlobal = Math.toDegrees(Math.atan2(
                alvoAtual.getY() - torreY,
                alvoAtual.getX() - torreX
        ));

        double anguloBaseTurret = anguloGlobal - headingGraus;

        double anguloGlobalRad = Math.atan2(
                alvoAtual.getY() - torreY,
                alvoAtual.getX() - torreX
        );

        while (anguloBaseTurret > 180) anguloBaseTurret -= 360;
        while (anguloBaseTurret <= -180) anguloBaseTurret += 360;


        // ====================================================================
        // 4. COMPENSAÇÃO DE VELOCIDADE (LEAD SHOT) E ATUAÇÃO
        // ====================================================================
        // O ShotSolution recebe os dados já calculados a partir da torre
        double compensacaoDinamica = ShotSolution.calcularCompensacao(shooter.getCurrentRPM(), distFinal, velAtual, anguloGlobalRad);

        double anguloFinal = anguloBaseTurret - compensacaoDinamica;

        turret.setAngle(anguloFinal);
        hood.setPositionFromDistance(distFinal);
        shooter.setRPMFromDistance(distFinal);

        // ====================================================================
        // 5. VALIDAÇÃO DE DISPARO RIGOROSA
        // ====================================================================
        double erroDeMira = Math.abs(turret.getCurrentAngle() - anguloFinal);

        boolean torreCravada = erroDeMira < 0.45;
        boolean motorPronto = shooter.isAtTargetRPM();

        // 6. O DISPARO
        if (torreCravada && motorPronto && (System.currentTimeMillis() - lastFireTime > FIRE_COOLDOWN_MS)) {
            new FireSequenceCommand(indexer, intake, hood).schedule();
            lastFireTime = System.currentTimeMillis();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hood.setOffsetTiro(0.0);
    }
}