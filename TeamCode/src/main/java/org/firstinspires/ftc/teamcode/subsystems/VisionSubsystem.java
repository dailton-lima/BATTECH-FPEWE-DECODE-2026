package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.List;

public class VisionSubsystem extends SubsystemBase {

    private Limelight3A limelight;

    // =========================================================================
    // CONSTANTES FÍSICAS DO SEU ROBÔ (POLEGADAS)
    // =========================================================================
    public static final double ALTURA_DA_CAMERA = 13.78;
    public static final double ALTURA_DO_ALVO = 29.5;
    public static final double ANGULO_DE_MONTAGEM = 28.0;
    // =========================================================================

    public VisionSubsystem(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
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
}