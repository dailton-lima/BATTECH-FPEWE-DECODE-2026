package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class Caminhos extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Define posição inicial
        Pose startPose = new Pose(32.1, 134.5, Math.toRadians(180)); //vc muda aq olhando a posição inicial do pedro pathing visualizer
        follower.setPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();

        Pose pose = follower.getPose();

        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));

        telemetry.update();
    }
}