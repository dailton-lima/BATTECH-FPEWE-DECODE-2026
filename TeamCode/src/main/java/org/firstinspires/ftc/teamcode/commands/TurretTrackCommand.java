package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.ShotSolution;

import java.util.function.Supplier;

public class TurretTrackCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;

    // Fornecedores dinâmicos de dados
    private final Supplier<Pose2d> targetPoseSupplier;
    private final Supplier<Integer> targetTagIdSupplier;

    private double cameraOffsetCorrection = 0;

    public TurretTrackCommand(TurretSubsystem turret, DriveSubsystem drive,
                              VisionSubsystem vision, ShooterSubsystem shooter, HoodSubsystem hood,
                              Supplier<Pose2d> targetPoseSupplier, Supplier<Integer> targetTagIdSupplier) {
        this.turret = turret;
        this.drive = drive;
        this.vision = vision;
        this.shooter = shooter;
        this.hood = hood;
        this.targetPoseSupplier = targetPoseSupplier;
        this.targetTagIdSupplier = targetTagIdSupplier;

        addRequirements(turret, hood);
    }

    @Override
    public void execute() {
        Pose2d poseAtual = drive.getPose();
        Pose2d velAtual = drive.getVelocity();

        // Pega a posição e a tag exatas para a aliança atual
        Pose2d alvoAtual = targetPoseSupplier.get();
        int tagAtiva = targetTagIdSupplier.get();

        // 1. Cálculo por Odometria
        double distOdo = Math.hypot(alvoAtual.getX() - poseAtual.getX(), alvoAtual.getY() - poseAtual.getY());
        double anguloGlobal = Math.toDegrees(Math.atan2(alvoAtual.getY() - poseAtual.getY(), alvoAtual.getX() - poseAtual.getX()));
        double anguloBaseTurret = anguloGlobal - poseAtual.getHeading();

        // 2. Compensação de Velocidade
        double compensacao = ShotSolution.calcularCompensacao(shooter.getCurrentRPM(), distOdo, velAtual);

        // 3. Ajuste fino pela Câmera (Filtra pela Tag correta!)
        if (vision.hasTarget(tagAtiva)) {
            cameraOffsetCorrection += (vision.getTx(tagAtiva) * 0.05);
        }

        // 4. Aplicação Contínua
        turret.setAngle(anguloBaseTurret - compensacao + cameraOffsetCorrection);
        hood.setAngleFromDistance(distOdo);
    }
}