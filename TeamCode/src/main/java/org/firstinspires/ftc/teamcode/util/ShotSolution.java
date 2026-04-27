package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.geometry.Pose2d;

public class ShotSolution {
    public static final double WHEEL_DIAMETER = 3.0;
    public static final double EFFICIENCY = 0.5;

    public static double calcularCompensacao(double currentRPM, double distanceToTarget, Pose2d robotVelocity) {
        double velocidadeReal = (currentRPM / 60.0) * (Math.PI * WHEEL_DIAMETER) * EFFICIENCY;
        if (velocidadeReal <= 0) return 0;

        double tempoDeVoo = distanceToTarget / velocidadeReal;

        // Compensação lateral baseada na velocidade em Y do robô (tangencial ao alvo)
        double desvioLateral = robotVelocity.getY() * tempoDeVoo;

        return Math.toDegrees(Math.atan2(desvioLateral, distanceToTarget));
    }
}