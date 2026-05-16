package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import java.util.Arrays;

@Config
public class HoodSubsystem extends SubsystemBase {

    private final Servo hoodServo;
    private final Telemetry telemetry;

    // Memória para exibir na telemetria
    private double currentPosition = 0.0;

    private double offsetTiro = 0.0;

    // =========================================================
    // A LUT: Distância (Polegadas) -> Posição Física do Servo (0.0 a 1.0)
    // =========================================================
    public final InterpLUT hoodLUT = new InterpLUT(
            // Distâncias em Polegadas
            // 50cm, 1m, 1.5m, 2m, 2.5m, 3m, 3.5m

            Arrays.asList(20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0, 160.0),

            // Posição correspondente do Servo (Ajuste fisicamente no Dashboard)
            // Exemplo: 0.1 (baixo) até 0.8 (alto)
            Arrays.asList(0.2, 0.4, 0.55, 0.8, 0.85, 0.8, 0.7, 0.65)
    );

    public HoodSubsystem(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        hoodServo = hwMap.get(Servo.class, "hoodServo");

        // Obrigatório para a interpolação matemática funcionar
        hoodLUT.createLUT();

        // Inicia o capô numa posição neutra/baixa
        setPosition(0.0);
    }

    /**
     * O MÉTODO INTELIGENTE:
     * Recebe a distância da Limelight (Polegadas), consulta a tabela e já move o servo.
     */

    public void setOffsetTiro(double offset) {
        this.offsetTiro = offset;
    }
    public void setPositionFromDistance(double distanceInches) {
        // Pega a posição bruta da tabela e SOMA o nosso desvio temporário
        double targetPos = hoodLUT.get(distanceInches) + offsetTiro;

        setPosition(targetPos);
    }

    /**
     * Método direto para mover o servo e registrar na telemetria
     */
    public void setPosition(double targetPosition) {
        // O Range.clip garante que se a tabela falhar ou for mal configurada,
        // o servo não tenta passar do seu limite elétrico, evitando queimas.
        double safePosition = Range.clip(targetPosition, 0.1, 1.0);

        this.currentPosition = safePosition;
        hoodServo.setPosition(safePosition);
        register();
    }

    public double getServoPosition() {
        return currentPosition;
    }

    // =========================================================
    // LOOP DE TELEMETRIA AUTOMÁTICO
    // =========================================================
    @Override
    public void periodic() {
        telemetry.addData("Hood - Posição Alvo LUT (0 a 1)", currentPosition);
        // Descomente abaixo se quiser ver o hardware real, mas costuma ser idêntico ao alvo
        // telemetry.addData("Hood - Posição Física Real", hoodServo.getPosition());
    }
}