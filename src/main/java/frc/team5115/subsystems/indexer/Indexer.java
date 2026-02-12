package frc.team5115.subsystems.indexer;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import frc.team5115.util.MotorContainer;
import java.util.ArrayList;
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
        return run(Constants.INDEX_VOMIT_SPEED);
    }

    @Override
    public ArrayList<SparkMax> getSparks() {
        return io.getSparks();
    }

    public boolean isIndexing() {
        return inputs.appliedVolts > 0.5;
    }

    public boolean isRejecting() {
        return inputs.appliedVolts < 0.5;
    }

    /**
     * @return true if either sensor detects something
     */
    public boolean isSensing() {
        return inputs.leftSensor && inputs.rightSensor;
    }
}
