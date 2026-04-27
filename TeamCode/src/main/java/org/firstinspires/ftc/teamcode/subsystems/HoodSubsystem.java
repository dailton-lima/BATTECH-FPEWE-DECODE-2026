package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT; // Importação da tabela

import java.util.Arrays;

@Config
public class HoodSubsystem extends SubsystemBase {

    private final Servo hoodServo;

    public static final double GEAR_RATIO = 10.0;
    public static final double SERVO_MAX_DEGREES = 200.0;
    public static final double HOOD_MAX_DEGREES = SERVO_MAX_DEGREES / GEAR_RATIO;

    public static double OFFSET = 0.0;

    // =========================================================
    // A LUT: Distância (metros/polegadas) -> Ângulo do Hood (graus)
    // =========================================================
    public final InterpLUT hoodLUT = new InterpLUT(
            // Distâncias reais da Limelight (Ex: 1m, 1.5m, 2m, 2.5m, 3m)
            Arrays.asList(1.0, 1.5, 2.0, 2.5, 3.0),

            // Ângulos do Hood para acertar o tiro nessas distâncias
            Arrays.asList(5.0, 8.5, 12.0, 15.5, 18.0)
    );

    public HoodSubsystem(HardwareMap hwMap) {
        hoodServo = hwMap.get(Servo.class, "hoodServo");

        // É obrigatório chamar o createLUT() no construtor para a matemática interna funcionar
        hoodLUT.createLUT();

        setAngle(0);
    }

    /**
     * O NOVO MÉTODO INTELIGENTE:
     * Recebe a distância da Limelight, consulta a tabela e já ajusta o servo.
     */
    public void setAngleFromDistance(double distance) {
        // Pega o valor interpolado na tabela
        double calculoDoAngulo = hoodLUT.get(distance);

        // Manda o capô para esse ângulo
        setAngle(calculoDoAngulo);
    }

    // O método original continua igual, para fazer a matemática do Servo
    public void setAngle(double targetHoodAngle) {
        double safeAngle = Range.clip(targetHoodAngle, 0, HOOD_MAX_DEGREES);
        double servoAngle = safeAngle * GEAR_RATIO;
        double servoPosition = (servoAngle / SERVO_MAX_DEGREES) + OFFSET;
        hoodServo.setPosition(servoPosition);
    }

    public double getServoPosition() {
        return hoodServo.getPosition();
    }
}