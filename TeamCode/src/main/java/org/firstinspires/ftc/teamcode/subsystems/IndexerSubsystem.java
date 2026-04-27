package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class IndexerSubsystem extends SubsystemBase {

    private final Servo gateServo;

    // Posições do Servo expostas no Dashboard para você testar
    public static double POS_LOCKED = 0.5;   // Posição que bloqueia a passagem
    public static double POS_UNLOCKED = 0.8; // Posição que libera a peça para o shooter

    public IndexerSubsystem(HardwareMap hwMap) {
        gateServo = hwMap.get(Servo.class, "gateServo");

        // Garante que o robô já inicie com a trava fechada
        lock();
    }

    public void lock() {
        gateServo.setPosition(POS_LOCKED);
    }

    public void unlock() {
        gateServo.setPosition(POS_UNLOCKED);
    }
}

