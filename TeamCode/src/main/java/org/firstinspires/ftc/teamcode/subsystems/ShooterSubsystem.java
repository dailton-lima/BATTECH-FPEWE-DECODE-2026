package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx launchermotor1;
    private final DcMotorEx launchermotor2;

    // Motores goBILDA SEM redução (Bare Motor) possuem exatos 28 ticks por revolução
    private final double TICKS_PER_REV = 28.0;

    // Variáveis expostas no Dashboard
    public static double targetRPM = 0.0;

    // Margem de erro (em RPM) aceitável para liberar o tiro.
    // Em 6000 RPM, flutuar 100 RPM é super normal.
    public static double RPM_TOLERANCE = 100.0;

    public ShooterSubsystem(HardwareMap hwMap) {
        launchermotor1 = hwMap.get(DcMotorEx.class, "shooterMotor1");
        launchermotor2 = hwMap.get(DcMotorEx.class, "shooterMotor2");

        // O modo RUN_USING_ENCODER liga o PIDF interno de Velocidade do Control/Expansion Hub.
        // Ele vai forçar a bateria a manter a velocidade constante automaticamente.
        launchermotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchermotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMPORTANTE: Dependendo da sua mecânica (se os motores estão no mesmo eixo,
        // ou um de cada lado da bola), um deles precisará girar ao contrário para ajudar o outro.
        launchermotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        launchermotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Define a velocidade do Flywheel em Rotações Por Minuto (RPM).
     */
    public void setTargetRPM(double rpm) {
        targetRPM = rpm;

        // O FTC SDK não entende RPM, ele entende "Ticks por Segundo" (TPS).
        // Fórmula: (RPM / 60) * Ticks_Por_Volta
        double ticksPerSecond = (rpm / 60.0) * TICKS_PER_REV;

        launchermotor1.setVelocity(ticksPerSecond);
        launchermotor2.setVelocity(ticksPerSecond);
    }

    /**
     * Retorna a velocidade atual lida pelo encoder do motor principal.
     */
    public double getCurrentRPM() {
        double currentTPS = launchermotor1.getVelocity();
        return (currentTPS / TICKS_PER_REV) * 60.0;
    }

    /**
     * Método super importante para o seu Fluxograma!
     * Verifica se o motor já acelerou e estabilizou na velocidade certa.
     */
    public boolean isAtTargetRPM() {
        // Ignora se o alvo for zero (não queremos atirar desligados)
        if (targetRPM == 0) return false;

        double erro = Math.abs(getCurrentRPM() - targetRPM);
        return erro <= RPM_TOLERANCE;
    }

    /**
     * Desliga os motores.
     */
    public void stop() {
        setTargetRPM(0);
    }

    @Override
    public void periodic() {
        // Como estamos usando o setVelocity() do próprio Hub da REV,
        // não precisamos fazer contas de PID manualmente aqui no periodic!
    }
}