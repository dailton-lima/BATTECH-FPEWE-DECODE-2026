package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem; // <-- Importação do Shooter

public class FireSequenceCommand extends SequentialCommandGroup {

    // Recebendo o Shooter no construtor
    public FireSequenceCommand(IndexerSubsystem indexer, IntakeSubsystem intake, ShooterSubsystem shooter) {

        addCommands(
                // 1. AÇÃO SIMULTÂNEA: Abre a trava E liga o intake ao mesmo tempo
                new ParallelCommandGroup(
                        new InstantCommand(() -> indexer.unlock(), indexer),
                        new InstantCommand(() -> intake.setPower(1.0), intake) // Força máxima pra empurrar a bola
                ),

                // 2. TEMPO DE ESPERA: Dá tempo para a bola passar pela trava e ser cuspida pelo shooter
                new WaitCommand(1200),

                // 3. AÇÃO SIMULTÂNEA: Fecha a trava, desliga o intake E desliga o shooter!
                new ParallelCommandGroup(
                        new InstantCommand(() -> indexer.lock(), indexer),
                        new InstantCommand(() -> intake.setPower(0.0), intake),
                        new InstantCommand(() -> shooter.stop(), shooter) // <-- Desliga o motor para poupar bateria
                )
        );
    }
}