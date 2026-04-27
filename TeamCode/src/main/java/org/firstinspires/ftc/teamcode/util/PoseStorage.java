package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.pedropathing.geometry.Pose;

public class PoseStorage {
    // Mantemos o valor estático
    public static Pose2d currentPose = new Pose2d(0, 0, 0);

    /**
     * Método utilitário para gravar a posição vinda do PedroPathing
     */
    public static void storePose(Pose pedroPose) {
        currentPose = new Pose2d(
                pedroPose.getX(),
                pedroPose.getY(),
                pedroPose.getHeading()
        );
    }
}