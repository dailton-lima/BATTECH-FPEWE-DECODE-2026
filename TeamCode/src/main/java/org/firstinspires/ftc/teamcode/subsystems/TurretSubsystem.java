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

    public static double ANGLE_OFFSET = 0.0;// <-- AJUSTE AQUI (negativo = corrige para esquerda)

    private final PIDController pidController = new PIDController(kP, kI, kD);

    private double targetAngle = 0.0;

    // =========================================================
    // VARIÁVEIS DA TRANSIÇÃO E CONTROLE
    // =========================================================
    private double teleopStartAngle = 0.0;
    private boolean usingAutoAngle = false;    // Evita o bug do "Offset Duplo"
    private boolean forcarZeroNoFinal = false; // A trava mecânica invisível

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

    /**
     * NOVO MÉTODO: Define a posição inicial vinda do PoseStorage
     */

    /**
     * NOVO MÉTODO: Ativa a trava invisível para o final do Autônomo
     */
    public void travarNoZero() {
        this.forcarZeroNoFinal = true;
        this.targetAngle = 0.0;
    }

    public void setAngle(double angle) {
        if (forcarZeroNoFinal) {
            // Se a trava estiver ativa, esmaga ordens externas e mantém em 0
            this.targetAngle = 0.0;
        } else {
            this.targetAngle = Range.clip(angle+ANGLE_OFFSET, -90, 90);
        }
    }

    public void setAngleOffset(double angleOffset) {
            this.ANGLE_OFFSET = angleOffset;
    }

    public double getCurrentAngle() {
        double rawAngle = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

        if (usingAutoAngle) {
            // O valor do autônomo JÁ TEM o offset físico embutido.
            // Basta somar ao encoder zerado.
            return rawAngle + teleopStartAngle;
        } else {
            // Se ligou o robô direto no TeleOp, usa a matemática padrão com offset.
            return rawAngle - ANGLE_OFFSET;
        }
    }

    @Override
    public void periodic() {
        pidController.setPID(kP, kI, kD);
        double pidOutput = pidController.calculate(getCurrentAngle(), targetAngle);

        double error = targetAngle - getCurrentAngle();
        double ffOutput = 0;

        if (Math.abs(error) > 0.5) {
            ffOutput = Math.signum(error) * kF;
        }

        double power = pidOutput + ffOutput;
        double safePower = Range.clip(power, -0.6, 0.6);

        // Salva o ângulo continuamente na memória estática

        turretMotor.setPower(safePower);

        telemetry.addData("Turret - Ângulo Alvo", targetAngle);
        telemetry.addData("Turret - Ângulo Real", getCurrentAngle());
        telemetry.addData("Turret - Erro (Graus)", error);

        // Linhas de Debug para ajudar no painel
        telemetry.addData("Turret - Herdado (Auto)?", usingAutoAngle ? "SIM (" + teleopStartAngle + ")" : "NAO");
        telemetry.addData("Turret - Trava do Zero?", forcarZeroNoFinal ? "ATIVA" : "Desligada");
    }
}