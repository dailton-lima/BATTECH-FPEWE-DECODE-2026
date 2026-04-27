package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotorEx intakeMotor;

    public IntakeSubsystem(HardwareMap hwMap) {
        // Altere "intakeMotor" para o nome exato que está no seu Driver Station
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");

        // Motores de intake geralmente não precisam de encoder, apenas força bruta
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // FLOAT para rolar livre quando desligar
    }

    /**
     * Define a força do motor do Intake.
     * @param power Força de -1.0 (reverso) a 1.0 (força total)
     */
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * Método de atalho para desligar o intake
     */
    public void stop() {
        intakeMotor.setPower(0);
    }
}