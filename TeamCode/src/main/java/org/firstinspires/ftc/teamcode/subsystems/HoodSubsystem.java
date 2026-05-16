package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import java.util.Arrays;

@Config
public class HoodSubsystem extends SubsystemBase {

    private final Servo hoodServo;
    private final Telemetry telemetry;

    private double currentPosition = 0.0;

    // =========================================================
    // PONTOS DA LUT (ajuste pelo Dashboard em tempo real)
    // Distâncias fixas em polegadas, só a posição do servo muda
    // =========================================================
    public static double POS_20  = 0.20;
    public static double POS_40  = 0.35;
    public static double POS_60  = 0.65;
    public static double POS_80  = 0.50;
    public static double POS_100 = 0.45;
    public static double POS_120 = 0.30;
    public static double POS_140 = 0.20;

    private double offsetTiro = 0.0;

    private InterpLUT hoodLUT;

    public HoodSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        hoodServo = hwMap.get(Servo.class, "hoodServo");
        buildLUT();
        setPosition(0.0);
        register();
    }

    /**
     * Constrói/reconstrói a LUT com os valores atuais das variáveis estáticas.
     */
    private void buildLUT() {
        hoodLUT = new InterpLUT(
                Arrays.asList(20.0,   40.0,   60.0,   80.0,   100.0,   120.0,   140.0),
                Arrays.asList(POS_20, POS_40, POS_60, POS_80, POS_100, POS_120, POS_140)
        );
        hoodLUT.createLUT();
    }

    /**
     * Usa a LUT para calcular a posição pela distância.
     */
    public void setOffsetTiro(double offset) {
        this.offsetTiro = offset;
    }

    public void setPositionFromDistance(double distanceInches) {
        double targetPos = hoodLUT.get(distanceInches);
        setPosition(targetPos);
    }



    /**
     * Move o servo diretamente para uma posição.
     */
    public void setPosition(double targetPosition) {
        double safePosition = Range.clip(targetPosition, 0.1, 1.0);
        this.currentPosition = safePosition;
        hoodServo.setPosition(safePosition);
    }

    public double getServoPosition() {
        return currentPosition;
    }

    @Override
    public void periodic() {
        // Reconstrói a LUT a cada loop com os valores atuais do Dashboard
        buildLUT();

        telemetry.addData("Hood - Posição Atual", currentPosition);
        telemetry.addData("Hood - POS_20",  POS_20);
        telemetry.addData("Hood - POS_40",  POS_40);
        telemetry.addData("Hood - POS_60",  POS_60);
        telemetry.addData("Hood - POS_80",  POS_80);
        telemetry.addData("Hood - POS_100", POS_100);
        telemetry.addData("Hood - POS_120", POS_120);
        telemetry.addData("Hood - POS_140", POS_140);
    }
}