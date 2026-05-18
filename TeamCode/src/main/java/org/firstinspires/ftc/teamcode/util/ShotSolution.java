package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.geometry.Pose2d;

@Config // Adicionado para você poder alterar os multiplicadores no Dashboard ao vivo!
public class ShotSolution {

    public static double WHEEL_DIAMETER = 3.0; // Polegadas

    // FÍSICA DO HOOD: Como a bola rola contra uma parede estática, perde 50% da velocidade linear.
    public static double HOOD_GEARING_FACTOR = 0.5;

    // Perda de energia por compressão da bola (Ajuste empírico entre 0.8 e 0.95)
    public static double SYSTEM_EFFICIENCY = 0.85;

    // A CHAVE MESTRA DO AJUSTE:
    // Compensa a resistência do ar. Se o robô anda para a direita e o tiro falha para a esquerda (atrasado), AUMENTE este valor.
    public static double TIME_OF_FLIGHT_MULTIPLIER = 1.35;

    /**
     * @param currentRPM RPM atual lido pelo encoder do motor.
     * @param distanceToTarget Distância em polegadas até o alvo.
     * @param robotVelocity Velocidade global do robô (Field-centric) devolvida pelo PedroPathing.
     * @param anguloGlobalAoAlvoRadianos O ângulo global (em radianos) da linha entre o robô e o cesto.
     */
    public static double calcularCompensacao(double currentRPM, double distanceToTarget, Pose2d robotVelocity, double anguloGlobalAoAlvoRadianos) {

        // 1. Extração Vetorial do PedroPathing
        double vx = robotVelocity.getX();
        double vy = robotVelocity.getY();

        // 2. Separação Vetorial (Radial vs Tangencial)
        // vRadial: Positivo se o robô anda NA DIREÇÃO do cesto, Negativo se afasta.
        double vRadialRobo = vx * Math.cos(anguloGlobalAoAlvoRadianos) + vy * Math.sin(anguloGlobalAoAlvoRadianos);

        // vTangencial: Velocidade de "escorregamento" lateral
        double vTangencialRobo = -vx * Math.sin(anguloGlobalAoAlvoRadianos) + vy * Math.cos(anguloGlobalAoAlvoRadianos);

        // 3. Velocidade Teórica de Saída (Polegadas por segundo)
        double velocidadeSuperficieRoda = (currentRPM / 60.0) * (Math.PI * WHEEL_DIAMETER);
        double velocidadeSaidaProjetil = velocidadeSuperficieRoda * HOOD_GEARING_FACTOR * SYSTEM_EFFICIENCY;

        // 4. Velocidade Relativa (A soma do tiro com o movimento frontal do robô)
        double velocidadeFinalFrente = velocidadeSaidaProjetil + vRadialRobo;

        // Proteção contra divisão por zero ou robô a fugir mais rápido que o tiro
        if (velocidadeFinalFrente <= 0) return 0;

        // 5. Tempo de Voo Corrigido (Com arrasto aerodinâmico)
        double tempoDeVoo = (distanceToTarget / velocidadeFinalFrente) * TIME_OF_FLIGHT_MULTIPLIER;

        // 6. Calcula a distância lateral que a bola vai "viajar" no ar devido à inércia do robô
        double desvioLateral = vTangencialRobo * tempoDeVoo;

        // 7. Retorna o ângulo para a torre antecipar o alvo
        return Math.toDegrees(Math.atan2(desvioLateral, distanceToTarget));
    }
}