package frc.team5115.subsystems.indexer;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5115.Constants;
import frc.team5115.util.MotorContainer;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase implements MotorContainer {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    private Command run(double speed) {
        return Commands.runEnd(() -> io.setPercent(speed), () -> io.setPercent(0), this);
    }

    /**
     * Index forever, stopping when interrupted.
     *
     * @return a RunEnd Command
     */
    public Command index() {
        return run(Constants.INDEX_SPEED);
    }

    /**
     * Reject forever, stopping when interrupted.
     *
     * @return a RunEnd Command
     */
    public Command reject() {
        return run(Constants.INDEX_REJECT_SPEED);
    }

    /**
     * Vomit forever, stopping when interrupted.
     *
     * @return a RunEnd Command
     */
    public Command vomit() {
        return agitate(Constants.INDEX_VOMIT_SPEED, 0, 0.25, 0.75);
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

    public Trigger debounceIsSensing() {
        return new Trigger(this::isSensing).debounce(2.0, Debouncer.DebounceType.kFalling);
    }

    @AutoLogOutput
    public boolean isIndexing() {
        return inputs.appliedVolts > 0.5;
    }

    @AutoLogOutput
    public boolean isRejecting() {
        return inputs.appliedVolts < 0.5;
    }

    /**
     * @return true if either sensor detects something
     */
    @AutoLogOutput
    public boolean isSensing() {
        return inputs.leftSensor && inputs.rightSensor;
    }
}
