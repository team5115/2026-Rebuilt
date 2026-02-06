package frc.team5115.subsystems.indexer;

import com.revrobotics.spark.SparkMax;
import frc.team5115.util.InterfaceReplayCheck;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IndexerIOInputs inputs) {
        InterfaceReplayCheck.warnOnNotReplay();
    }

    /** Run the indexer motor at the specified voltage. */
    public default void setVoltage(double volts) {
        InterfaceReplayCheck.warnOnNotReplay();
    }

    /** Run the indexer motor at the specified percentage. */
    public default void setPercent(double percent) {
        InterfaceReplayCheck.warnOnNotReplay();
    }

    /** This should not really be here but it must be */
    public default ArrayList<SparkMax> getSparks() {
        InterfaceReplayCheck.warnOnNotReplay();
        return new ArrayList<>();
    }
}
