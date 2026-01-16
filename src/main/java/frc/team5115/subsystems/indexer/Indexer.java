package frc.team5115.subsystems.indexer;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
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

    public Command setSpeed(double speed) {
        return Commands.runOnce(() -> io.setPercent(speed));
    }

    public Command index() {
        return setSpeed(Constants.INDEX_SPEED);
    }

    public Command vomit() {
        return setSpeed(-Constants.INDEX_SPEED);
    }

    public Command stop() {
        return setSpeed(0);
    }

    public void getSparks(ArrayList<SparkMax> sparks) {
        io.getSparks(sparks);
    }
}
