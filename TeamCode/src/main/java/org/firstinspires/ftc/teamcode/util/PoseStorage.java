package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.pedropathing.geometry.Pose;

public class PoseStorage {

    // =========================================================
    // 1. CHASSIS (PedroPathing)
    // =========================================================
    public static Pose currentPose = new Pose(0, 0, 0);

    public static void storePose(Pose pedroPose) {
        currentPose = pedroPose;
    }

    public static Pose getPose() {
        return currentPose;
    }

    // =========================================================
    // 2. TORRETA (Turret)
    // =========================================================
    public static double currentTurretAngle = 0.0;

    public static void storeTurretAngle(double angle) {
        currentTurretAngle = angle;
    }

    public static double getTurretAngle() {
        return currentTurretAngle;
    }
}