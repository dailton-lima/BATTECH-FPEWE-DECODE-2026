package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem extends SubsystemBase {

    // APAGAMOS OS DCMOTORS! O PedroPathing já os possui.
    private final VoltageSensor battery;
    private final Follower follower;
    private final Telemetry telemetry;

    public DriveSubsystem(HardwareMap hw, VoltageSensor battery, Follower follower, Telemetry telemetry) {
        this.battery = battery;
        this.follower = follower;
        this.telemetry = telemetry;

        // Não há mais hw.get() nem setDirection() aqui.
        // O Constants.java já resolveu isso na inicialização do Follower.

        register();
    }

    private double compensar(double p) {
        return Math.max(-1, Math.min(1, p * (13.0 / battery.getVoltage())));
    }

    public void resetHeading() {
        // Mantém as coordenadas X e Y de onde o robô está, mas zera a "bússola"
        follower.setStartingPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
    }

    public void drive(double x, double y, double rot) {
        // Aplica a compensação de bateria nos inputs
        //double compX = compensar(x);
        //double compY = compensar(y);
        //double compRot = compensar(rot);

        // ATENÇÃO: A nova arquitetura exige que você INICIE o modo TeleOp
        // antes de enviar os vetores. Caso contrário, ele ignora os comandos.
        // NOVO MÉTODO DO PEDROPATHING 1.0+
        // Parâmetros: (Forward, Strafe, Turn, FieldCentric?)
        follower.setTeleOpDrive(y, x, rot, true, 0);
    }

    public Pose2d getPose() {
        return new Pose2d(follower.getPose().getX(), follower.getPose().getY(),
                new Rotation2d(follower.getPose().getHeading()));
    }

    public Pose2d getVelocity() {
        Vector v = follower.getVelocity();
        return new Pose2d(v.getXComponent(), v.getYComponent(), new Rotation2d(0));
    }

    @Override
    public void periodic() {
        // Agora o update() do PedroPathing é o ÚNICO que envia sinais para os motores. Paz no hardware!
        follower.update();

        telemetry.addData("Drive - Posição X", follower.getPose().getX());
        telemetry.addData("Drive - Posição Y", follower.getPose().getY());
        telemetry.addData("Drive - Orientação (Graus)", Math.toDegrees(follower.getPose().getHeading()));
    }
}