package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.geometry.Pose2d;

public class ShotSolution {
    public static final double WHEEL_DIAMETER = 3.0; // Polegadas
    // A eficiência geralmente cai um pouco em altas velocidades, 0.5 a 0.7 é um bom palpite empírico
    public static final double EFFICIENCY = 0.5;

    /**
     * @param currentRPM RPM atual lido pelo encoder do motor.
     * @param distanceToTarget Distância em polegadas até o alvo.
     * @param robotVelocity Velocidade global do robô (Field-centric) devolvida pelo PedroPathing.
     * @param anguloGlobalAoAlvoRadianos O ângulo global (em radianos) da linha imaginária entre o robô e o cesto.
     */
    public static double calcularCompensacao(double currentRPM, double distanceToTarget, Pose2d robotVelocity, double anguloGlobalAoAlvoRadianos) {

        // 1. Velocidade de saída do projétil
        double velocidadeReal = (currentRPM / 60.0) * (Math.PI * WHEEL_DIAMETER) * EFFICIENCY;
        if (velocidadeReal <= 0) return 0;

        // 2. Tempo de voo
        double tempoDeVoo = distanceToTarget / velocidadeReal;

        // 3. A MÁGICA VETORIAL: Extrai apenas a velocidade tangencial (lateral) ao alvo
        // Isso resolve o problema de atirar enquanto anda na diagonal!
        double vx = robotVelocity.getX();
        double vy = robotVelocity.getY();

        // Produto cruzado 2D (Rotaciona o vetor de velocidade para alinhar com o olhar da torre)
        double velocidadeTangencial = -vx * Math.sin(anguloGlobalAoAlvoRadianos) + vy * Math.cos(anguloGlobalAoAlvoRadianos);

        // 4. Calcula o quanto o anel vai "escorregar" de lado durante o voo
        double desvioLateral = velocidadeTangencial * tempoDeVoo;

        // 5. Converte essa distância de escorregamento num ângulo para a torre compensar
        return Math.toDegrees(Math.atan2(desvioLateral, distanceToTarget));
    }
}