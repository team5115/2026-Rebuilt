package frc.team5115.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import frc.team5115.util.InterfaceReplayCheck;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double positionRad = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {
        InterfaceReplayCheck.warnOnNotReplay();
    }

    public default void setVoltage(double volts) {
        InterfaceReplayCheck.warnOnNotReplay();
    }

    public default ArrayList<SparkMax> getSparks() {
        InterfaceReplayCheck.warnOnNotReplay();
        return new ArrayList<>();
    }
}
