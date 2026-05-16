package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

@Config
public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx launchermotor1;
    private final DcMotorEx launchermotor2;

    private final double TICKS_PER_REV = 28.0;

    public static double targetRPM = 0.0;
    public static double RPM_TOLERANCE = 300.0;

    // =========================================================
    // A LUT DO SHOOTER: Distância (pol) -> RPM
    // =========================================================
    public final InterpLUT shooterLUT = new InterpLUT();

    private final Telemetry telemetry;

    public ShooterSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        launchermotor1 = hwMap.get(DcMotorEx.class, "shooterMotor1");
        launchermotor2 = hwMap.get(DcMotorEx.class, "shooterMotor2");

        launchermotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchermotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launchermotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        launchermotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // CONFIGURAÇÃO DA TABELA (Exemplo de valores)
        // Adicione seus pontos aqui: .add(distância, rpm)
        shooterLUT.add(20.0, 3200.0);
        shooterLUT.add(40.0, 3600.0);
        shooterLUT.add(60.0, 3750.0);
        shooterLUT.add(80.0, 3900.0);
        shooterLUT.add(100.0, 4050.0);
        shooterLUT.add(120.0, 4200.0);
        shooterLUT.add(140.0, 4350.0);
        shooterLUT.add(160.0, 4500.0);
        shooterLUT.createLUT();

        register();
    }

    /**
     * Calcula o RPM ideal baseado na distância e o define como alvo.
     */
    public void setRPMFromDistance(double distanceInches) {
        double rpm = shooterLUT.get(distanceInches);
        setTargetRPM(rpm);
    }

    public void setTargetRPM(double rpm) {
        // Trava de segurança: impede RPM negativo ou acima do limite do motor
        targetRPM = Range.clip(rpm, 0, 6000);

        double ticksPerSecond = (targetRPM / 60.0) * TICKS_PER_REV;

        launchermotor1.setVelocity(ticksPerSecond);
        launchermotor2.setVelocity(ticksPerSecond);
    }


    public double getCurrentRPM() {
        return (launchermotor1.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    public boolean isAtTargetRPM() {
        if (targetRPM <= 500) return false;
        return Math.abs(getCurrentRPM() - targetRPM) <= RPM_TOLERANCE;
    }

    public void setPIDF(double p, double i, double d, double f) {
        launchermotor1.setVelocityPIDFCoefficients(p, i, d, f);
        launchermotor2.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void stop() {
        setTargetRPM(0);
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter - Alvo (RPM)", targetRPM);
        telemetry.addData("Shooter - Real (RPM)", getCurrentRPM());
        telemetry.addData("Shooter - Pronto?", isAtTargetRPM() ? "SIM" : "NAO");
    }
}