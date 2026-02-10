package frc.team5115.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import frc.team5115.util.InterfaceReplayCheck;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double position = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {
        InterfaceReplayCheck.warnOnNotReplay();
    }

    /** Run the shooter motor at the specified voltage. */
    public default void setVoltage(double volts) {
        InterfaceReplayCheck.warnOnNotReplay();
    }

    public default ArrayList<SparkMax> getSparks() {
        InterfaceReplayCheck.warnOnNotReplay();
        return new ArrayList<>();
    }
}
