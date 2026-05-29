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

    public static double ANGLE_OFFSET = 0.0; // <-- AJUSTE AQUI (negativo = corrige para esquerda)

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

    public void travarNoZero() {
        this.forcarZeroNoFinal = true;
        this.targetAngle = 0.0;
    }

    public void setAngle(double angle) {
        if (forcarZeroNoFinal) {
            // Se a trava estiver ativa, esmaga ordens externas e mantém em 0
            this.targetAngle = 0.0;
        } else {
            // CORREÇÃO: Removemos o + ANGLE_OFFSET daqui!
            // Guardamos apenas a intenção LÓGICA pura que o comando ShootOnMove pediu.
            this.targetAngle = angle;
        }
    }

    public void setAngleOffset(double angleOffset) {
        this.ANGLE_OFFSET = angleOffset;
    }

    public double getCurrentAngle() {
        double rawAngle = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

        if (usingAutoAngle) {
            return rawAngle + teleopStartAngle;
        } else {
            // O ShootOnMove recebe o ângulo subtraído do offset, 
            // logo ele vê o mundo através da matemática pura, sem saber que o motor está compensado.
            return rawAngle - ANGLE_OFFSET;
        }
    }

    @Override
    public void periodic() {
        pidController.setPID(kP, kI, kD);
        
        // CORREÇÃO: O Offset é somado APENAS no cálculo Físico do PID!
        // O Alvo Físico real da torre considera o Alvo Matemático + Offset do Piloto.
        // O Range.clip garante que o offset não force a torre a partir os limites físicos de 90 graus.
        double physicalTarget = Range.clip(targetAngle + ANGLE_OFFSET, -90, 90);
        
        // O Posição Física real (leitura bruta do motor)
        double physicalCurrent = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

        // O PID agora calcula usando valores 100% físicos, acabando com a ilusão!
        double pidOutput = pidController.calculate(physicalCurrent, physicalTarget);

        double error = physicalTarget - physicalCurrent;
        double ffOutput = 0;

        if (Math.abs(error) > 0.5) {
            ffOutput = Math.signum(error) * kF;
        }

        double power = pidOutput + ffOutput;
        double safePower = Range.clip(power, -0.6, 0.6);

        turretMotor.setPower(safePower);

        // Telemetria Ajustada para mostrar o que está acontecendo por trás das cortinas
        telemetry.addData("Turret - Alvo Matemático", targetAngle);
        telemetry.addData("Turret - Alvo Físico (Motor)", physicalTarget);
        telemetry.addData("Turret - Offset Atual", ANGLE_OFFSET);
        telemetry.addData("Turret - Erro Físico", error);
        telemetry.addData("Turret - Herdado (Auto)?", usingAutoAngle ? "SIM (" + teleopStartAngle + ")" : "NAO");
        telemetry.addData("Turret - Trava do Zero?", forcarZeroNoFinal ? "ATIVA" : "Desligada");
    }
}
