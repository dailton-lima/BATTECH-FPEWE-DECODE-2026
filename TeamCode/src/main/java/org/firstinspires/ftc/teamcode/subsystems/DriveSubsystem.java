package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import com.seattlesolvers.solverslib.command.SubsystemBase; // Importação necessária
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Adicionado o extends SubsystemBase para funcionar com os Commands
public class DriveSubsystem extends SubsystemBase {

    DcMotor fe, fd, te, td;
    VoltageSensor battery;
    IMU imu;

    // Quando você instanciar o PedroPathing, ele entrará aqui
    // public Follower follower;

    public DriveSubsystem(HardwareMap hw, VoltageSensor battery) {
        this.battery = battery;

        fe = hw.get(DcMotor.class, "FE");
        fd = hw.get(DcMotor.class, "FD");
        te = hw.get(DcMotor.class, "TE");
        td = hw.get(DcMotor.class, "TD");

        fe.setDirection(DcMotorSimple.Direction.REVERSE);
        te.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hw.get(IMU.class, "imu");

        // Verifique se essa orientação bate com a posição física do seu Hub!
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }

    private double compensar(double p) {
        return Math.max(-1, Math.min(1, p * (13.0 / battery.getVoltage())));
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public void drive(double x, double y, double rot) {
        // Pega o ângulo para rotacionar os vetores do joystick
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotação de vetores para Field-Oriented
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Cinemática Mecanum
        double feP = rotY + rotX + rot;
        double fdP = rotY - rotX - rot;
        double teP = rotY - rotX + rot;
        double tdP = rotY + rotX - rot;

        // Normalização para não passar de 1.0 mantendo a proporção
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rot), 1);

        fe.setPower(compensar(feP / denominator));
        fd.setPower(compensar(fdP / denominator));
        te.setPower(compensar(teP / denominator));
        td.setPower(compensar(tdP / denominator));
    }

    /**
     * Retorna a posição do robô. No futuro, use:
     * return new Pose2d(follower.getPose().getX(), follower.getPose().getY(),
     * new Rotation2d(follower.getPose().getHeading()));
     */

    public Pose2d getPose() {
        // Exemplo de como plugar o PedroPathing
        return new Pose2d(follower.getPose().getX(), follower.getPose().getY(),
                new Rotation2d(follower.getPose().getHeading()));
    }

    /**
     * Retorna a velocidade do robô. Crucial para o cálculo do ShotSolution!
     * No futuro, use: return follower.getVelocity();
     */
    /**
     * Retorna a velocidade do robô. Crucial para o cálculo do ShotSolution!
     */
    public Pose2d getVelocity() {
        // 1. Pega a velocidade vetorial (Vector do PedroPathing)
        // Se a palavra Vector ficar vermelha, clique nela e dê Alt+Enter para importar!
        Vector v = follower.getVelocity();

        // 2. Extrai os componentes X e Y do vetor
        double velX = v.getXComponent();
        double velY = v.getYComponent();

        // 3. Devolve exatamente o que o ShootOnMove precisa (Pose2d da SolversLib)
        return new Pose2d(velX, velY, new Rotation2d(0));
    }
}