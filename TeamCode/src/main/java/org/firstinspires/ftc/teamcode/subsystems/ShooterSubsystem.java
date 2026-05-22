package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

@Config
public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx launchermotor1;
    private final DcMotorEx launchermotor2;
    private final VoltageSensor voltageSensor;

    private final double TICKS_PER_REV = 28.0;

    // =========================================================
    // RPM — visíveis e editáveis no Dashboard
    // =========================================================
    public static double targetRPM     = 0.0;
    public static double RPM_TOLERANCE = 75.0; // Tolerância mais justa graças ao PID suave

    // =========================================================
    // CALIBRAÇÃO MANUAL pelo Dashboard
    // =========================================================
    public static double  SHOOTER_RPM_TEST = 0.0;
    public static boolean ENABLE_TEST_RPM  = false;

    // =========================================================
    // RAMPA DE ACELERAÇÃO
    // =========================================================
    public static boolean ENABLE_RAMP    = false;
    public static double  RAMP_RATE_UP   = 300.0;
    public static double  RAMP_RATE_DOWN = 50.0;
    private double rampedRPM = 0.0;

    // =========================================================
    // PIDF EXTERNO E FILTRO PASSA-BAIXA
    // NOTA: Como agora calculamos Power (0 a 1.0) em vez de Ticks,
    // as constantes são decimais bem pequenos!
    // =========================================================
    public static double MOTOR_P = 0.0006;  // Correção de erros
    public static double MOTOR_I = 0.0000;  // Quase nunca usado em shooters
    public static double MOTOR_D = 0.0001;  // Amortece oscilações
    public static double MOTOR_F = 0.00016; // (1.0 Power / 6000 RPM) -> Constante base (kV)

    public static double TENSAO_NOMINAL = 13.0; // Tensão na qual o MOTOR_F foi calibrado

    // Suavizador de leitura do encoder (0.0 = ignora tudo, 1.0 = confia cegamente no sensor)
    public static double FILTER_ALPHA = 0.8;
    private double filteredRPM = 0.0;

    private final ElapsedTime pidTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;

    // =========================================================
    // LUT: Distância (pol) -> RPM  (editável pelo Dashboard)
    // =========================================================
    public static double RPM_20  = 3150.0;
    public static double RPM_40  = 3300.0;
    public static double RPM_60  = 4000.0;
    public static double RPM_80  = 4000.0;
    public static double RPM_100 = 4150.0;
    public static double RPM_110 = 4300.0;
    public static double RPM_120 = 4550.0;
    public static double RPM_140 = 4700.0;
    public static double RPM_160 = 4800.0;

    private InterpLUT shooterLUT;
    private final Telemetry telemetry;

    public ShooterSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        launchermotor1 = hwMap.get(DcMotorEx.class, "shooterMotor1");
        launchermotor2 = hwMap.get(DcMotorEx.class, "shooterMotor2");
        voltageSensor  = hwMap.voltageSensor.iterator().next();

        // MUDANÇA CRUCIAL: Retiramos o poder do REV Hub. Nós somos o PID agora.
        launchermotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchermotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launchermotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        launchermotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        launchermotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launchermotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        targetRPM = 0.0;
        pidTimer.reset();

        buildLUT();
        register();
    }

    private void buildLUT() {
        shooterLUT = new InterpLUT(
                Arrays.asList( 20.0,    40.0,    60.0,    80.0,    100.0,   110.0,   120.0,   140.0,   160.0),
                Arrays.asList(RPM_20, RPM_40, RPM_60, RPM_80, RPM_100, RPM_110, RPM_120, RPM_140, RPM_160)
        );
        shooterLUT.createLUT();
    }

    // =========================================================
    // API PÚBLICA
    // =========================================================
    public void setRPMFromDistance(double distanceInches) {
        setTargetRPM(shooterLUT.get(distanceInches));
    }

    public void setTargetRPM(double rpm) {
        targetRPM = Range.clip(rpm, 0, 6000);
    }

    public double getCurrentRPM() {
        return (launchermotor1.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    public boolean isAtTargetRPM() {
        if (targetRPM <= 500) return false;
        // Agora usamos o RPM filtrado e suave para validar o disparo
        return Math.abs(filteredRPM - targetRPM) <= RPM_TOLERANCE;
    }

    public void stop() {
        setTargetRPM(0);
    }

    // =========================================================
    // PERIODIC: O Coração do PIDF Customizado
    // =========================================================
    @Override
    public void periodic() {
        buildLUT();

        // 1. Leitura de Sensores e Filtro
        double rawRPM = getCurrentRPM();
        double tensaoAtual = voltageSensor.getVoltage();

        // Aplica o Filtro Passa-Baixa (Suaviza a vibração mecânica na leitura)
        filteredRPM = (FILTER_ALPHA * rawRPM) + ((1.0 - FILTER_ALPHA) * filteredRPM);

        // 2. Determinação do Alvo e Rampa
        double activeTarget = ENABLE_TEST_RPM ? Range.clip(SHOOTER_RPM_TEST, 0, 6000) : targetRPM;

        if (ENABLE_RAMP) {
            if (rampedRPM < activeTarget) rampedRPM = Math.min(rampedRPM + RAMP_RATE_UP, activeTarget);
            else rampedRPM = Math.max(rampedRPM - RAMP_RATE_DOWN, activeTarget);
        } else {
            rampedRPM = activeTarget;
        }

        // 3. Cálculos de Tempo para o PID
        double dt = pidTimer.seconds();
        pidTimer.reset();

        // Proteção contra ciclos bizarros (0 ou muito grandes)
        if (dt <= 0.0 || dt > 0.1) dt = 0.02;

        double powerFinal = 0.0;

        // Se o alvo for zero, limpa a memória do PID e corta os motores
        if (rampedRPM <= 10) {
            integralSum = 0;
            lastError = 0;
            powerFinal = 0.0;
        } else {
            // 4. O CÁLCULO DO PIDF
            double error = rampedRPM - filteredRPM;

            // Proporcional
            double p = MOTOR_P * error;

            // Integral (com Anti-Windup limitando o acúmulo de I)
            integralSum += error * dt;
            double maxIntegral = 0.3 / (MOTOR_I + 0.000001); // Limita o I a no máximo 30% de power
            integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
            double i = MOTOR_I * integralSum;

            // Derivativo
            double derivative = (error - lastError) / dt;
            double d = MOTOR_D * derivative;
            lastError = error;

            // Feedforward DINÂMICO (A Mágica da Tensão)
            double fBase = MOTOR_F * rampedRPM;
            double fatorCompensacao = TENSAO_NOMINAL / tensaoAtual;
            double fCompensado = fBase * fatorCompensacao;

            // Soma final do Power
            powerFinal = p + i + d + fCompensado;
        }

        // 5. Envia para o motor (Sempre limitado entre 0 e 1.0 para segurança)
        powerFinal = Range.clip(powerFinal, 0.0, 1.0);
        launchermotor1.setPower(powerFinal);
        launchermotor2.setPower(powerFinal); // Assumindo que o setDirection já inverteu este corretamente

        // 6. Telemetria Rica para Calibração
        telemetry.addLine("=== SHOOTER PIDF EXTERNO ===");
        telemetry.addData("Bateria", String.format("%.2f V", tensaoAtual));
        telemetry.addData("Alvo  (RPM)", activeTarget);
        telemetry.addData("Real (Bruto)", rawRPM);
        telemetry.addData("Real (Filtrado)", filteredRPM);
        telemetry.addData("Erro  (RPM)", rampedRPM - filteredRPM);
        telemetry.addData("Power Output", String.format("%.3f", powerFinal));
        telemetry.addData("Pronto?", isAtTargetRPM() ? "SIM" : "NAO");
    }
}