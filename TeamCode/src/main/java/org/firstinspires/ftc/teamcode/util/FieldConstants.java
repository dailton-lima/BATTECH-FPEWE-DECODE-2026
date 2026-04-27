package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.geometry.Pose2d;

public class FieldConstants {
    public enum Alliance { BLUE, RED }
    public enum TargetGoal { GOAL }

    // Esta variável pode ser definida no início do TeleOp ou vinda do PoseStorage
    public static Alliance activeAlliance = Alliance.BLUE;

    // Definição da posição do GOAL com base na aliança
    public static Pose2d getTargetPose(TargetGoal goal) {
        if (activeAlliance == Alliance.BLUE) {
            return new Pose2d(0, 0, 0); // Coordenada do GOAL Azul
        } else {
            return new Pose2d(144, 0, 0); // Coordenada do GOAL Vermelho
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