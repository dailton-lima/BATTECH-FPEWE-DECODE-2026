package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TurretSubsystem extends SubsystemBase {

    private final DcMotorEx turretMotor;

    public static double kP = 0.052;
    public static double kI = 0.0;
    public static double kD = 0.0009;
    public static double kF = 0.05; // Feedforward estático

    private final PIDController pidController = new PIDController(kP, kI, kD);

    private double targetAngle = 0.0;

    // Correção da divisão decimal aqui
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

    public void setAngle(double angle) {
        this.targetAngle = Range.clip(angle, -90, 90);
    }

    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    @Override
    public void periodic() {
        pidController.setPID(kP, kI, kD);
        double pidOutput = pidController.calculate(getCurrentAngle(), targetAngle);

        double error = targetAngle - getCurrentAngle();
        double ffOutput = 0;

        // Aplica o feedforward apenas se o erro for maior que meio grau
        if (Math.abs(error) > 0.5) {
            ffOutput = Math.signum(error) * kF;
        }

        double power = pidOutput + ffOutput;
        double safePower = Range.clip(power, -0.6, 0.6);

        turretMotor.setPower(safePower);

        telemetry.addData("Turret - Ângulo Alvo", targetAngle);
        telemetry.addData("Turret - Ângulo Real", getCurrentAngle());

        // Mostrar o erro ajuda muito: se ficar perto de 0, a torre está perfeita
        double erro = targetAngle - getCurrentAngle();
        telemetry.addData("Turret - Erro (Graus)", erro);

        // Ver se o limite de hardware está sendo ativado
        telemetry.addData("Turret - Limite de Segurança", (getCurrentAngle() >= 80 || getCurrentAngle() <= -80) ? "ATIVO" : "Livre");
    }
}