package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
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

    // Fornecedores dinâmicos do FieldConstants
    private final Supplier<Pose2d> targetPoseSupplier;
    private final Supplier<Integer> targetTagIdSupplier;

    // Variável para guardar o erro acumulado entre odometria e visão
    private double cameraOffsetCorrection = 0;
    private boolean dadosValidos = false;

    public ShootOnMoveCommand(TurretSubsystem turret, DriveSubsystem drive, VisionSubsystem vision,
                              ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake, HoodSubsystem hood,
                              Supplier<Pose2d> targetPoseSupplier, Supplier<Integer> targetTagIdSupplier) {
        this.turret = turret;
        this.drive = drive;
        this.vision = vision;
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.hood = hood;
        this.targetPoseSupplier = targetPoseSupplier;
        this.targetTagIdSupplier = targetTagIdSupplier;

        addRequirements(turret, shooter, hood);
    }

    @Override
    public void initialize() {
        dadosValidos = false;
        cameraOffsetCorrection = 0; // Reseta o erro acumulado ao iniciar
        shooter.setTargetRPM(6000);
    }

    @Override
    public void execute() {
        // Pega o Alvo e a Tag corretos para este exato momento da partida
        Pose2d alvoAtual = targetPoseSupplier.get();
        int tagAtiva = targetTagIdSupplier.get();

        // 1. DADOS DE ODOMETRIA (Sem Delay)
        Pose2d poseAtual = drive.getPose();
        Pose2d velAtual = drive.getVelocity();

        // 2. CÁLCULO DE DISTÂNCIA E ÂNGULO TEÓRICO (MAPA)
        double distFinal = poseAtual.getTranslation().getDistance(alvoAtual.getTranslation());

        if (vision.hasTarget(tagAtiva)) {
            double distCamera = vision.getDistance(tagAtiva);
            distFinal = (distCamera * 0.8) + (distFinal * 0.2);
        }
        // =========================================================================

        // Ângulo absoluto do robô para o cesto
        double anguloGlobal = Math.toDegrees(Math.atan2(
                alvoAtual.getY() - poseAtual.getY(),
                alvoAtual.getX() - poseAtual.getX()
        ));

        // Ângulo que a turret deveria estar relativo ao robô
        double anguloBaseTurret = anguloGlobal - poseAtual.getHeading();

        // 3. COMPENSAÇÃO DE VELOCIDADE (Lead Shot)
        double compensacaoDinamica = ShotSolution.calcularCompensacao(shooter.getCurrentRPM(), distFinal, velAtual);

        // 4. CHECK DA CÂMERA (Correção de Drift - Horizontal)
        if (vision.hasTarget(tagAtiva)) {
            double tx = vision.getTx(tagAtiva);
            cameraOffsetCorrection += (tx * 0.1);
        }

        // 5. CÁLCULO FINAL
        double anguloFinal = anguloBaseTurret - compensacaoDinamica + cameraOffsetCorrection;

        // 6. ATUAÇÃO
        turret.setAngle(anguloFinal);
        hood.setAngleFromDistance(distFinal);

        // 7. VALIDAÇÃO PARA DISPARO
        double erroDeMira = Math.abs(turret.getCurrentAngle() - anguloFinal);

        // Se a câmera estiver vendo, usa o tx DELA para validar
        if (vision.hasTarget(tagAtiva)) {
            erroDeMira = Math.abs(vision.getTx(tagAtiva));
        }

        dadosValidos = (erroDeMira < 1.5) && shooter.isAtTargetRPM();
    }

    @Override
    public boolean isFinished() {
        return dadosValidos;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            new FireSequenceCommand(indexer, intake, shooter).schedule();
        } else {
            shooter.stop();
        }
    }
}