package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry; // Importação da Telemetria
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.List;

public class VisionSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private final Telemetry telemetry; // Variável para a telemetria

    // =========================================================================
    // CONSTANTES FÍSICAS DO SEU ROBÔ (POLEGADAS)
    // =========================================================================
    public static final double ALTURA_DA_CAMERA = 13.78;
    public static final double ALTURA_DO_ALVO = 29.5;
    public static final double ANGULO_DE_MONTAGEM = 28.0;
    // =========================================================================

    // Construtor atualizado exigindo a Telemetry
    public VisionSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        register();
    }

    /**
     * Verifica se uma AprilTag específica está visível na tela.
     */
    public boolean hasTarget(int targetId) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    if (fr.getFiducialId() == targetId) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /**
     * Retorna o erro horizontal (Tx) apenas da AprilTag solicitada.
     */
    public double getTx(int targetId) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    if (fr.getFiducialId() == targetId) {
                        return fr.getTargetXDegrees();
                    }
                }
            }
        }
        return 0.0;
    }

    /**
     * Retorna a distância trigonométrica calculada a partir do Ty da AprilTag solicitada.
     */
    public double getDistance(int targetId) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    if (fr.getFiducialId() == targetId) {

                        // Pega o ângulo vertical (Ty) específico desta AprilTag
                        double ty = fr.getTargetYDegrees();

                        double anguloTotalEmGraus = ANGULO_DE_MONTAGEM + ty;
                        double anguloRadianos = Math.toRadians(anguloTotalEmGraus);

                        return (ALTURA_DO_ALVO - ALTURA_DA_CAMERA) / Math.tan(anguloRadianos);
                    }
                }
            }
        }
        return 0.0;
    }

    /**
     * Método de fallback genérico (opcional, caso queira rastrear a cor principal em vez de Tag)
     */
    public boolean hasTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    // =========================================================
    // LOOP DE TELEMETRIA AUTOMÁTICO
    // =========================================================
    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            // Verifica se há alguma AprilTag visível
            if (fiducials != null && !fiducials.isEmpty()) {
                // Pega a primeira AprilTag que a câmera estiver vendo para mostrar na tela
                LLResultTypes.FiducialResult fr = fiducials.get(0);

                int id = fr.getFiducialId();
                double ty = fr.getTargetYDegrees();

                // Faz a mesma matemática do getDistance() para exibir em tempo real
                double anguloTotalEmGraus = ANGULO_DE_MONTAGEM + ty;
                double anguloRadianos = Math.toRadians(anguloTotalEmGraus);
                double distanciaCalculada = (ALTURA_DO_ALVO - ALTURA_DA_CAMERA) / Math.tan(anguloRadianos);

                telemetry.addData("Vision - Tag Ativa (ID)", id);
                telemetry.addData("Vision - Distância (Polegadas)", distanciaCalculada);
                telemetry.addData("Vision - Alinhamento Tx", fr.getTargetXDegrees());
            } else {
                telemetry.addData("Vision", "Buscando AprilTags...");
            }
        } else {
            telemetry.addData("Vision", "Offline / Sem Resultados");
        }
    }
}