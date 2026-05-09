package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class FireSequenceCommand extends SequentialCommandGroup {

    // Removemos o Shooter daqui! Ele não pertence a esta sequência.
    public FireSequenceCommand(IndexerSubsystem indexer, IntakeSubsystem intake) {

        addCommands(
                // 1. AÇÃO SIMULTÂNEA: Abre a trava E liga o intake ao mesmo tempo
                new SequentialCommandGroup(
                        new InstantCommand(() -> indexer.unlock(), indexer),
                        new WaitCommand(400),
                        new InstantCommand(() -> intake.setPower(1.0), intake)
                ),

                // 2. TEMPO DE ESPERA: Tempo para a bola ser cuspida (Reduzido para 0.5s)
                new WaitCommand(2000),

                // 3. AÇÃO SIMULTÂNEA: Fecha a trava e desliga o intake (O Shooter continua ligado!)
                new ParallelCommandGroup(
                        new InstantCommand(() -> indexer.lock(), indexer),
                        new InstantCommand(() -> intake.stop(), intake)
                )
        );
    }
}