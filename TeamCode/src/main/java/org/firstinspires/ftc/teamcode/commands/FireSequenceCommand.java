package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class FireSequenceCommand extends SequentialCommandGroup {

    public FireSequenceCommand(IndexerSubsystem indexer, IntakeSubsystem intake, HoodSubsystem hood) {

        addCommands(
                // 1. AÇÃO SIMULTÂNEA: Abre a trava e liga o intake para empurrar as bolas
                new SequentialCommandGroup(

                        new InstantCommand(() -> indexer.unlock(), indexer),
                        new WaitCommand(150),
                        new InstantCommand(() -> intake.setPower(1.0), intake),


                // 2. Espera o tempo exato para a PRIMEIRA BOLA ser arremessada
                        new WaitCommand(50),

                // 3. RECOLHE O HOOD: Aplica o offset de -0.15 (Ajuste para +0.15 se o servo for invertido)
                        new InstantCommand(() -> hood.setOffsetTiro(-0.1)),

                // 4. Espera as outras duas bolas saírem (Resto do tempo da rajada)
                        new WaitCommand(700),

                // 5. FIM DA RAJADA: Fecha trava, para o intake e RESETA O CAPÔ
                        new ParallelCommandGroup(
                                new InstantCommand(() -> indexer.lock(), indexer),
                                new InstantCommand(() -> intake.stop(), intake),
                                new InstantCommand(() -> hood.setOffsetTiro(0.0)) // <--- Volta a mira ao normal!
                    )
                )
        );
    }
}