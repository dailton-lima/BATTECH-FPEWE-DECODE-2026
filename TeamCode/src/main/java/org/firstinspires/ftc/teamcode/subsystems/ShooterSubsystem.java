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

@Config
public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx launchermotor1;
    private final DcMotorEx launchermotor2;

    private final double TICKS_PER_REV = 28.0;

    // =========================================================
    // RPM
    // =========================================================
    public static double targetRPM = 0.0;
    private double currentRPM = 0.0;
    public static double RPM_TOLERANCE = 300.0;

    // =========================================================
    // RAMPA DE ACELERAÇÃO (ajuste pelo Dashboard)
    // =========================================================
    public static double RAMP_RATE_UP = 300.0;   // RPM por loop subindo
    public static double RAMP_RATE_DOWN = 50.0;  // RPM por loop descendo (mais suave)

    // =========================================================
    // PIDF INTERNO DO MOTOR (GoBilda 5203 312RPM)
    // Ajuste pelo Dashboard em tempo real
    // =========================================================
    public static double MOTOR_P = 15.0;
    public static double MOTOR_I = 0.0;
    public static double MOTOR_D = 0.0;
    public static double MOTOR_F = 13.0;

    // =========================================================
    // LUT: Distância (pol) -> RPM
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

        launchermotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launchermotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // =========================================================
        // TABELA DE DISTÂNCIA -> RPM
        // =========================================================
        shooterLUT.add(20.0,  3150.0);
        shooterLUT.add(40.0,  3300.0);
        shooterLUT.add(60.0,  3400.0);
        shooterLUT.add(80.0,  3550.0);
        shooterLUT.add(100.0, 3700.0);
        shooterLUT.add(120.0, 3850.0);
        shooterLUT.add(140.0, 4100.0);
        shooterLUT.add(160.0, 4300.0);
        shooterLUT.createLUT();

        //launchermotor1.setVelocityPIDFCoefficients(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F);
        //launchermotor2.setVelocityPIDFCoefficients(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F);

        register();
    }

    /**
     * Calcula o RPM ideal baseado na distância e o define como alvo.
     */
    public void setRPMFromDistance(double distanceInches) {
        double rpm = shooterLUT.get(distanceInches);
        setTargetRPM(rpm);
    }

    /**
     * Define o RPM alvo — a rampa no periodic() cuida da aceleração suave.
     */
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

    public void stop() {
        setTargetRPM(0);
    }

    @Override
    public void periodic() {
        // =========================================================
        // RAMPA DE ACELERAÇÃO/DESACELERAÇÃO
        // =========================================================
//        if (currentRPM < targetRPM) {
//            currentRPM = Math.min(currentRPM + RAMP_RATE_UP, targetRPM);
//        } else if (currentRPM > targetRPM) {
//            currentRPM = Math.max(currentRPM - RAMP_RATE_DOWN, targetRPM);
//        }

        // =========================================================
        // TELEMETRIA
        // =========================================================
        telemetry.addData("Shooter - Alvo (RPM)",  targetRPM);
        telemetry.addData("Shooter - Rampa (RPM)", currentRPM);
        telemetry.addData("Shooter - Real (RPM)",  getCurrentRPM());
        telemetry.addData("Shooter - Pronto?",     isAtTargetRPM() ? "SIM" : "NAO");
    }
}