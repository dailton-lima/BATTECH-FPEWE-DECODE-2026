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

    public static double kP = 0.052;
    public static double kI = 0.0;
    public static double kD = 0.0009;
    public static double kF = 0.05;
    public static double ANGLE_OFFSET = 0.0; // <-- AJUSTE AQUI (negativo = corrige para esquerda)

    private final PIDController pidController = new PIDController(kP, kI, kD);

    private double targetAngle = 0.0;

    // NOVA VARIÁVEL: Guarda a posição herdada do Autônomo
    private double teleopStartAngle = 0.0;

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
     * Atualiza o desvio da telemetria e o targetAngle para impedir que o PID atue.
     */
    public void loadStartingAngle(double storedAngle) {
        this.teleopStartAngle = storedAngle;
        this.targetAngle = Range.clip(storedAngle, -90, 90);
    }

    public void setAngle(double angle) {
        this.targetAngle = Range.clip(angle, -90, 90);
    }

    public double getCurrentAngle() {
        // O valor real agora é a leitura do motor zerado + offset + o ângulo guardado do autônomo
        return (turretMotor.getCurrentPosition() / TICKS_PER_DEGREE) - ANGLE_OFFSET + teleopStartAngle;
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

        PoseStorage.storeTurretAngle(getCurrentAngle());

        turretMotor.setPower(safePower);

        telemetry.addData("Turret - Ângulo Alvo", targetAngle);
        telemetry.addData("Turret - Ângulo Real", getCurrentAngle());
        telemetry.addData("Turret - Offset Físico", ANGLE_OFFSET);
        telemetry.addData("Turret - Offset Autônomo", teleopStartAngle);
        telemetry.addData("Turret - Erro (Graus)", error);
        telemetry.addData("Turret - Limite de Segurança", (getCurrentAngle() >= 80 || getCurrentAngle() <= -80) ? "ATIVO" : "Livre");
    }
}