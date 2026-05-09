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

    private final Supplier<Pose2d> targetPoseSupplier;
    private final Supplier<Integer> targetTagIdSupplier;

    // Constante Proporcional (kP) para a Câmera
    // Se a torre tremer, diminua este valor. Se demorar a focar, aumente.
    private static final double VISION_KP = 0.4;

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

        Pose2d alvoAtual = targetPoseSupplier.get();
        int tagAtiva = targetTagIdSupplier.get();

        // ========================================================
        // 1. CÁLCULO DE ODOMETRIA (Distância e Ângulo)
        // ========================================================
        double deltaX = alvoAtual.getX() - poseAtual.getX();
        double deltaY = alvoAtual.getY() - poseAtual.getY();

        double distOdo = Math.hypot(deltaX, deltaY);
        double anguloGlobalGraus = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Pega a rotação do chassi (Assume que o getHeading() retorna Radianos, padrão do PedroPathing)
        // Se o seu getHeading já estiver devolvendo graus, apague o Math.toDegrees()
        double headingChassiGraus = Math.toDegrees(poseAtual.getHeading());

        // Ângulo base que a torre precisa virar
        double anguloBaseTurret = anguloGlobalGraus - headingChassiGraus;

        // NORMALIZAÇÃO DE ÂNGULO (O Segredo para não travar!)
        // Transforma ângulos absurdos (ex: 270) no caminho mais curto (ex: -90)
        while (anguloBaseTurret > 180) anguloBaseTurret -= 360;
        while (anguloBaseTurret <= -180) anguloBaseTurret += 360;

        // ========================================================
        // 2. COMPENSAÇÃO DE VELOCIDADE (Lead Shot)
        // ========================================================

        double anguloGlobalRad = Math.atan2(
                alvoAtual.getY() - poseAtual.getY(),
                alvoAtual.getX() - poseAtual.getX()
        );

        double compensacao = ShotSolution.calcularCompensacao(shooter.getCurrentRPM(), distOdo, velAtual, anguloGlobalRad);

        // ========================================================
        // 3. AJUSTE FINO PELA CÂMERA
        // ========================================================
        double cameraOffsetCorrection = 0; // REINICIA A CADA LOOP (Fim do Bug do Windup)

        if (vision.hasTarget(tagAtiva)) {
            // Usa APENAS o sinal de Igual (=). O erro é lido no momento e multiplicado pela força (kP)
            cameraOffsetCorrection = vision.getTx(tagAtiva) * VISION_KP;
        }

        // ========================================================
        // 4. APLICAÇÃO FINAL (Turret & Hood)
        // ========================================================
        double anguloFinal = anguloBaseTurret - compensacao + cameraOffsetCorrection;

        turret.setAngle(anguloFinal);
        hood.setPositionFromDistance(distOdo);
    }
}