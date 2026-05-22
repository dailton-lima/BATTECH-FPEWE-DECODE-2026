package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range; // <-- Novo Import Obrigatório
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.ShotSolution;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretTrackCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    private final HoodSubsystem hood;

    private final Supplier<Pose2d> targetPoseSupplier;
    private final Supplier<Integer> targetTagIdSupplier;

    // Multiplicador de distância para o hood.
    // TeleOp: 1.0 (sem alteração)
    // Autônomo: ajuste até o hood bater na posição correta (ex: 0.7)

    // Constante Proporcional (kP) para a Câmera
    private static final double VISION_KP = 0.4;

    private final DoubleSupplier rpmSupplier;

    // Construtor padrão (TeleOp) — multiplier = 1.0, sem mudar nada

    // Construtor com multiplicador (Autônomo)
    public TurretTrackCommand(TurretSubsystem turret, DriveSubsystem drive,
                              VisionSubsystem vision, HoodSubsystem hood,
                              Supplier<Pose2d> targetPoseSupplier, Supplier<Integer> targetTagIdSupplier,
                              DoubleSupplier rpmSupplier) {
        this.turret = turret;
        this.drive = drive;
        this.vision = vision;
        this.hood = hood;
        this.targetPoseSupplier = targetPoseSupplier;
        this.targetTagIdSupplier = targetTagIdSupplier;
        this.rpmSupplier = rpmSupplier;

        addRequirements(turret, hood);
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
        double rpmAtual = rpmSupplier.getAsDouble();
        double compensacaoDinamica = 0.0;

        // SÓ aplica a física de compensação de movimento se o motor estiver ligado!
        // Se estiver abaixo de 500 RPM, ignora para evitar bugs matemáticos de divisão por zero.
        if (rpmAtual > 500) {
            compensacaoDinamica = ShotSolution.calcularCompensacao(rpmAtual, distFinal, velAtual, anguloGlobalRad);
        }

        double anguloFinal = anguloBaseTurret - compensacaoDinamica;

        turret.setAngle(anguloFinal);
        hood.setPositionFromDistance(distFinal);

//        // ====================================================================
//        // 5. O PULO DO GATO: COMPENSAÇÃO DINÂMICA DO HOOD NO AUTÔNOMO
//        // ====================================================================
//        // Acessa o alvo que foi definido externamente no subsistema
//        double targetRPM = ShooterSubsystem.targetRPM;
//
//        // Só aplica a correção do Hood se o robô tiver intenção de atirar
//        if (targetRPM > 2000) {
//            double rpmError = targetRPM - rpmAtual;
//
//            // Multiplicador de Compensação (Mesmo valor que calibrar no ShootOnMove)
//            double HOOD_COMP_kP = 0.0002;
//
//            // Limita o offset para impedir danos mecânicos no servo (-15% a +15%)
//            double offsetDinamico = Range.clip(rpmError * HOOD_COMP_kP, -0.05, 0.05);
//
//            hood.setOffsetTiro(offsetDinamico);
//        } else {
//            // Se o motor está desligado (ex: o robô está a viajar para a próxima pose), zera o offset
//            hood.setOffsetTiro(0.0);
//        }
    }
}