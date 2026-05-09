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

    // Timer interno para não "metralhar" o comando do indexador no CommandScheduler
    private long lastFireTime = 0;
    private static final long FIRE_COOLDOWN_MS = 500; // Meio segundo entre tiros

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
        this.telemetry = telemetry; // <--- Salva a telemetria

        // IMPORTANTE: Este comando não deve dar "require" no Indexer nem no Intake
        addRequirements(turret, shooter, hood);
    }

    @Override
    public void initialize() {
        shooter.setTargetRPM(4000);
    }

    @Override
    public void execute() {
        Pose2d alvoAtual = targetPoseSupplier.get();
        int tagAtiva = targetTagIdSupplier.get();

        Pose2d poseAtual = drive.getPose();
        Pose2d velAtual = drive.getVelocity();

        // ====================================================================
        // 1. DISTÂNCIA 100% CÂMERA (Com salva-vidas de Odometria)
        // ====================================================================
        double distFinal;

        double distOdo = Math.hypot(alvoAtual.getX() - poseAtual.getX(), alvoAtual.getY() - poseAtual.getY());
        double distCamera = 0;

        if (vision.hasTarget(tagAtiva)) {
            // Confia plenamente na câmera (com o seu offset de 12 polegadas)
            distCamera = 12 + vision.getDistance(tagAtiva);
            distFinal = distCamera;
        } else {
            // Fallback: Se a câmera piscar, usa o mapa
            distFinal = distOdo;
        }

        // ====================================================================
        // TELEMETRIA DE DEBUG (Mostra os dois valores na tela)
        // ====================================================================
        telemetry.addData("MIRA - Distância Odometria", distOdo);
        telemetry.addData("MIRA - Distância Limelight", vision.hasTarget(tagAtiva) ? distCamera : "Sem Alvo");
        telemetry.addData("MIRA - Distância Final Usada", distFinal);

        // ====================================================================
        // 2. MATEMÁTICA DA TORRE (Odometria Base)
        // ====================================================================
        double anguloGlobal = Math.toDegrees(Math.atan2(
                alvoAtual.getY() - poseAtual.getY(),
                alvoAtual.getX() - poseAtual.getX()
        ));

        double headingGraus = Math.toDegrees(poseAtual.getHeading());
        double anguloBaseTurret = anguloGlobal - headingGraus;

        double anguloGlobalRad = Math.atan2(
                alvoAtual.getY() - poseAtual.getY(),
                alvoAtual.getX() - poseAtual.getX()
        );

        // Angle Wrap
        while (anguloBaseTurret > 180) anguloBaseTurret -= 360;
        while (anguloBaseTurret <= -180) anguloBaseTurret += 360;

        // ====================================================================
        // 3. SOBREPOSIÇÃO DA CÂMERA (Matemática Absoluta - Fim do erro de 5 graus)
        // ====================================================================
//        if (vision.hasTarget(tagAtiva)) {
//            // Descobre a posição real do alvo somando onde a torre está com o erro (tx)
//            double anguloAbsolutoDaTag = turret.getCurrentAngle() + vision.getTx(tagAtiva);
//            anguloBaseTurret = anguloAbsolutoDaTag;
//        }

        // ====================================================================
        // 4. COMPENSAÇÃO DE VELOCIDADE (LEAD SHOT) E ATUAÇÃO
        // ====================================================================
        double compensacaoDinamica = ShotSolution.calcularCompensacao(shooter.getCurrentRPM(), distFinal, velAtual, anguloGlobalRad);

        double anguloFinal = anguloBaseTurret - compensacaoDinamica;
        turret.setAngle(anguloFinal);
        hood.setPositionFromDistance(distFinal);
        shooter.setRPMFromDistance(distFinal); // <--- ADICIONE ESTA LINHA!

        // ====================================================================
        // 5. VALIDAÇÃO DE DISPARO RIGOROSA
        // ====================================================================
        double erroDeMira = Math.abs(turret.getCurrentAngle() - anguloFinal);

        boolean torreCravada = erroDeMira < 0.5;
        boolean motorPronto = shooter.isAtTargetRPM();

        // 6. O DISPARO (Agora com a trava da 'torreCravada' ativa!)
        if (torreCravada && motorPronto && (System.currentTimeMillis() - lastFireTime > FIRE_COOLDOWN_MS)) {

            new FireSequenceCommand(indexer, intake).schedule();
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
    }
}