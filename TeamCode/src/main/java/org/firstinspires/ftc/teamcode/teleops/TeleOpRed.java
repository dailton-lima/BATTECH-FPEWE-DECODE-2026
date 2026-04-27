package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

@TeleOp(name = "🔵 TeleOp VERMELHO", group = "Competição")
public class TeleOpRed extends TeleOpCommandBased {

    @Override
    public void initialize() {
        // 1. Define a aliança cravada como AZUL
        FieldConstants.activeAlliance = FieldConstants.Alliance.RED;

        // 2. Opcional: Se quiser dar um reset de segurança na posição
        // PoseStorage.currentPose = new Pose2d(0,0,0);

        // 3. Roda todo aquele seu código gigante do TeleOpCommandBased
        super.initialize();
    }
}