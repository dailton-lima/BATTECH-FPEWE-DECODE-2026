package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Config
public class FireSequenceCommand extends SequentialCommandGroup {

    public static double OFFSET_TIRO_1 = -0.0125;
    public static double OFFSET_TIRO_2 = -0.02;

    public FireSequenceCommand(IndexerSubsystem indexer, IntakeSubsystem intake, HoodSubsystem hood) {

        addCommands(
                new SequentialCommandGroup(

                        new InstantCommand(() -> indexer.unlock(), indexer),
                        new WaitCommand(300),
                        new InstantCommand(() -> intake.setPower(1.0), intake),

                        new WaitCommand(200),

                        new InstantCommand(() -> hood.setOffsetTiro(OFFSET_TIRO_1)),

                        new WaitCommand(400),

                        new InstantCommand(() -> hood.setOffsetTiro(OFFSET_TIRO_2)),

                        new WaitCommand(400),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> indexer.lock(), indexer),
                                new InstantCommand(() -> intake.stop(), intake),
                                new InstantCommand(() -> hood.setOffsetTiro(0.0))
                        )
                )
        );
    }
}