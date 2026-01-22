package frc.team5115.subsystems.agitator;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.subsystems.MotorContainer;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Agitator extends SubsystemBase implements MotorContainer {
    private final AgitatorIO io;
    private final AgitatorIOInputsAutoLogged inputs = new AgitatorIOInputsAutoLogged();

    public Agitator(AgitatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Agitator", inputs);
    }

    /**
     * Run slow forever
     *
     * @return a Run Command
     */
    public Command slow() {
        return run(0.2);
    }

    /**
     * Run fast forever
     *
     * @return a Run Command
     */
    public Command fast() {
        return run(1.0);
    }

    private Command run(double speed) {
        return Commands.run(
                () -> {
                    io.setPercent(speed);
                },
                this);
    }

    @Override
    public void getSparks(ArrayList<SparkMax> sparks) {
        io.getSparks(sparks);
    }
}
