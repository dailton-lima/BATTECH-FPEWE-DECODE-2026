package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.geometry.Pose2d;

public class FieldConstants {
    public enum Alliance { BLUE, RED }
    public enum TargetGoal { GOAL }

    // Esta variável pode ser definida no início do TeleOp ou vinda do PoseStorage
    public static Alliance activeAlliance = Alliance.BLUE;

    public static Pose2d poseGoalAzul = new Pose2d(9.0, 133.0, 0.0);

    public static Pose2d poseGoalVermelho = new Pose2d(133.0, 133.0, 0.0);

    // Definição da posição do GOAL com base na aliança
    public static Pose2d getTargetPose(TargetGoal goal) {
        if (activeAlliance == Alliance.BLUE) {
            return poseGoalAzul; // Coordenada do GOAL Azul
        } else {
            return poseGoalVermelho; // Coordenada do GOAL Vermelho
        }
    }

    // Definição da AprilTag do GOAL com base na aliança
    public static int getTargetTagId(TargetGoal goal) {
        if (activeAlliance == Alliance.BLUE) {
            return 20; // ID da AprilTag do GOAL Azul
        } else {
            return 24; // ID da AprilTag do GOAL Vermelho
        }
    }
}