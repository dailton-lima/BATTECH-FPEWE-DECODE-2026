package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Config
public class TurretSubsystem extends SubsystemBase {

    private final DcMotorEx turretMotor;

    public static double kP = 0.05;
    public static double kI = 0.0;
    public static double kD = 0.0009;
    public static double kF = 0.05;

    private final PIDController pidController = new PIDController(kP, kI, kD);

    private double targetAngle = 0.0;

    // =========================================================
    // VARIÁVEIS DA TRANSIÇÃO E CONTROLE
    // =========================================================
    public boolean isManualMode = false;       // <-- NOVO: Controle de calibração manual
    private double teleopStartAngle = 0.0;
    private boolean usingAutoAngle = false;    // Herdado do Autônomo
    private boolean forcarZeroNoFinal = false; // Trava mecânica invisível

    private final double TICKS_PER_REV_MOTOR = 537.7;
    private final double EXTERNAL_GEAR_RATIO = 140.0 / 30.0;
    private final double TICKS_PER_DEGREE = (TICKS_PER_REV_MOTOR * EXTERNAL_GEAR_RATIO) / 360.0;

    private final Telemetry telemetry;

    public TurretSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        register();
    }

    public void travarNoZero() {
        this.forcarZeroNoFinal = true;
        this.targetAngle = 0.0;
    }

    public void setAngle(double angle) {
        if (forcarZeroNoFinal) {
            // Se a trava estiver ativa, esmaga ordens externas e mantém em 0
            this.targetAngle = 0.0;
        } else if (!isManualMode) {
            // Só aceita ordens matemáticas se NÃO estiver no modo manual (Fail-Safe)
            this.targetAngle = Range.clip(angle, -90, 90);
        }
    }

    /**
     * Zera o encoder fisicamente na posição atual.
     * O que quer que seja a posição atual da torre, passa a ser o novo 0 graus.
     */
    public void resetEncoder() {
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.targetAngle = 0.0; // Define o alvo para o novo zero
    }

    /**
     * Define a potência bruta do motor (usado APENAS no modo manual)
     */
    public void setManualPower(double power) {
        if (isManualMode) {
            turretMotor.setPower(Range.clip(power, -0.4, 0.4)); // Limite seguro
        }
    }

    public double getCurrentAngle() {
        double rawAngle = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

        if (usingAutoAngle) {
            return rawAngle + teleopStartAngle;
        } else {
            return rawAngle; // O encoder já é a verdade absoluta agora
        }
    }

    @Override
    public void periodic() {
        // Se estivermos em modo manual, o PID é IGNORADO.
        if (isManualMode) {
            telemetry.addData("Turret Mode", "MANUAL (Ajuste o Zero!)");
            telemetry.addData("Turret - Ângulo Bruto", getCurrentAngle());
            return; // Sai do método antes de calcular o PID
        }

        // --- CÁLCULO NORMAL DO PID (Modo Automático) ---
        pidController.setPID(kP, kI, kD);
        
        double physicalTarget = Range.clip(targetAngle, -90, 90);
        double physicalCurrent = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

        double pidOutput = pidController.calculate(physicalCurrent, physicalTarget);

        double error = physicalTarget - physicalCurrent;
        double ffOutput = 0;

        if (Math.abs(error) > 0.5) {
            ffOutput = Math.signum(error) * kF;
        }

        double power = pidOutput + ffOutput;
        double safePower = Range.clip(power, -0.6, 0.6);

        turretMotor.setPower(safePower);

        telemetry.addData("Turret Mode", "AUTO TRACKING");
        telemetry.addData("Turret - Alvo Matemático", targetAngle);
        telemetry.addData("Turret - Real", getCurrentAngle());
        telemetry.addData("Turret - Erro Físico", error);
        telemetry.addData("Turret - Herdado (Auto)?", usingAutoAngle ? "SIM (" + teleopStartAngle + ")" : "NAO");
        telemetry.addData("Turret - Trava do Zero?", forcarZeroNoFinal ? "ATIVA" : "Desligada");
    }
}
