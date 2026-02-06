package frc.team5115.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import frc.team5115.util.InterfaceReplayCheck;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {
        InterfaceReplayCheck.warnOnNotReplay();
    }

    /** Run the intake motor at the specified voltage. */
    public default void setVoltage(double volts) {
        InterfaceReplayCheck.warnOnNotReplay();
    }

    /** Run the intake motor at the specified percentage. */
    public default void setPercent(double percent) {
        InterfaceReplayCheck.warnOnNotReplay();
    }

    /** This should not really be here but it must be */
    public default ArrayList<SparkMax> getSparks() {
        InterfaceReplayCheck.warnOnNotReplay();
        return new ArrayList<>();
    }
}
