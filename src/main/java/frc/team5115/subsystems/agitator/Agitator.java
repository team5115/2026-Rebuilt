package frc.team5115.subsystems.agitator;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import frc.team5115.util.MotorContainer;
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
        return run(Constants.AGITATOR_SLOW_SPEED);
    }

    /**
     * Run fast forever
     *
     * @return a Run Command
     */
    public Command fast() {
        return agitate(
                Constants.AGITATOR_FAST_SPEED,
                Constants.AGITATOR_ALT_SPEED,
                Constants.AGITATOR_MAIN_PAUSE,
                Constants.AGITATOR_ALT_PAUSE);
    }

    /**
     * Run the motor backward
     *
     * @return a Run Command
     */
    public Command vomit() {
        return run(Constants.AGITATOR_VOMIT_SPEED);
    }

    private Command run(double speed) {
        return Commands.runOnce(() -> io.setPercent(speed), this).andThen(Commands.idle(this));
    }

    private Command agitate(double speed1, double speed2, double pause1, double pause2) {
        return Commands.sequence(
                        setSpeed(speed1),
                        Commands.waitSeconds(pause1),
                        setSpeed(speed2),
                        Commands.waitSeconds(pause2))
                .repeatedly()
                .finallyDo(() -> io.setPercent(0));
    }

    private Command setSpeed(double speed) {
        return Commands.runOnce(() -> io.setPercent(speed), this);
    }

    @Override
    public ArrayList<SparkMax> getSparks() {
        return io.getSparks();
    }
}
