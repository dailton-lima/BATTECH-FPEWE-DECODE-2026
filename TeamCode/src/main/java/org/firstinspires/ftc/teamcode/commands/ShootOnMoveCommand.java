package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range; // <-- Import necessário para travar o limite do servo
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
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
    private final boolean isAutonomous;
    private final boolean shoot;
    private boolean tiroIniciado = false;
    private FireSequenceCommand comandoDeTiro = null;

    public ShootOnMoveCommand(TurretSubsystem turret, DriveSubsystem drive, VisionSubsystem vision,
                              ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake, HoodSubsystem hood,
                              Supplier<Pose2d> targetPoseSupplier, Supplier<Integer> targetTagIdSupplier, Telemetry telemetry,
                              boolean isAutonomous, boolean shoot) {
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
        this.isAutonomous = isAutonomous;
        this.shoot = shoot;

        addRequirements(turret, shooter, hood);
    }

    @Override
    public void initialize() {
        // Reseta o estado do tiro toda vez que o comando é chamado/agendado
        tiroIniciado = false;
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
        double OFFSET_X = -2.0;
        double OFFSET_Y = 0.0;

        // Calculates real position of the turret on the field
        double torreX = poseAtual.getX() + (OFFSET_X * Math.cos(headingRad) - OFFSET_Y * Math.sin(headingRad));
        double torreY = poseAtual.getY() + (OFFSET_X * Math.sin(headingRad) + OFFSET_Y * Math.cos(headingRad));

        // ====================================================================
        // 1. DISTÂNCIA 100% ODOMETRIA (Usando as coordenadas da Torre!)
        // ====================================================================
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
        double currentRPM = shooter.getCurrentRPM();
        double compensacaoDinamica = ShotSolution.calcularCompensacao(currentRPM, distFinal, velAtual, anguloGlobalRad);
        double anguloFinal = anguloBaseTurret - compensacaoDinamica;

        turret.setAngle(anguloFinal);

        // Define as posições base teóricas
        hood.setPositionFromDistance(distFinal);
        shooter.setRPMFromDistance(distFinal);

        // ====================================================================
        // 4.5. O PULO DO GATO: COMPENSAÇÃO DINÂMICA DO HOOD
        // ====================================================================
        // Pega o RPM que o motor DEVERIA estar vs o RPM que ele ESTÁ agora
        double targetRPM = ShooterSubsystem.targetRPM; // Acessa a variável estática do seu subsistema
        double rpmError = targetRPM - currentRPM;

        // Multiplicador de Compensação (Ajuste no laboratório!)
//        // Ex: Se o erro for 100 RPM e o Kp for 0.0004, o offset do servo será 0.04
//        double HOOD_COMP_kP = 0.0002;
//
//        double offsetDinamico = rpmError * HOOD_COMP_kP;
//
//        // Trava de segurança: impede que o servo tente quebrar o robô se o erro for absurdo
//        offsetDinamico = Range.clip(offsetDinamico, -0.05, 0.05);
//
//        // Aplica o offset em tempo real enquanto o robô se move
//        hood.setOffsetTiro(offsetDinamico);

        // ====================================================================
        // 5. VALIDAÇÃO DE DISPARO ACELERADA E FLEXÍVEL
        // ====================================================================
        double erroDeMira = Math.abs(turret.getCurrentAngle() - anguloFinal);

        // A sua tolerância ajustada
        double toleranciaAceitavel = (distFinal < 100.0) ? 1.7 : 0.5;

        boolean torreCravada = erroDeMira < toleranciaAceitavel;

        // O robô não espera mais o erro ficar minúsculo. Se estiver dentro de 250 RPM, ele atira
        // sabendo que o Hood compensou a diferença mecânica.
        boolean motorPronto = Math.abs(rpmError) < 250.0;

        // Telemetria de diagnóstico em movimento
//        telemetry.addData("HOOD - Offset Dinamico", offsetDinamico);
        telemetry.addData("MIRA - Erro Angular Atual", erroDeMira);
        telemetry.addData("MIRA - Tolerancia Permitida", toleranciaAceitavel);
        telemetry.addData("MIRA - Torre Travada?", torreCravada ? "SIM" : "NAO");
        telemetry.addData("MIRA - Motor Aceitavel?", motorPronto ? "SIM" : "NAO");

        // ====================================================================
        // 6. O DISPARO
        // ====================================================================
        if (shoot && torreCravada && motorPronto && !tiroIniciado && (System.currentTimeMillis() - lastFireTime > FIRE_COOLDOWN_MS)) {
            comandoDeTiro = new FireSequenceCommand(indexer, intake, hood);
            comandoDeTiro.schedule();
            lastFireTime = System.currentTimeMillis();

            tiroIniciado = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (!isAutonomous) {
            return false; // No TeleOp, não termina sozinho
        } else {
            // Se já mandou atirar...
            if (tiroIniciado) {
                // Espera 100ms para garantir que o agendador registrou o comando de tiro
                if (System.currentTimeMillis() - lastFireTime > 100) {
                    // Só termina quando a sequência de tiro sumir da fila de comandos!
                    if (comandoDeTiro != null && !comandoDeTiro.isScheduled()) {
                        return true;
                    }
                }
            }
            // Salva-vidas: Se algo travar e ele não conseguir mirar,
            // aborta após 4 segundos para não estragar o resto do Autônomo.
            if (System.currentTimeMillis() - lastFireTime > 4000 && !tiroIniciado) {
                return true;
            }
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hood.setOffsetTiro(0.0);
    }
}