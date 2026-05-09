package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

@TeleOp(name = "🔵 TeleOp AZUL", group = "Competição")
public class TeleOpBlue extends TeleOpCommandBased {

    @Override
    public void initialize() {
        // 1. Define a aliança cravada como AZUL
        FieldConstants.activeAlliance = FieldConstants.Alliance.BLUE;

        // 3. Roda todo aquele seu código gigante do TeleOpCommandBased

        super.initialize();
    }
}