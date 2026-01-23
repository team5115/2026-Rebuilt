package frc.team5115.subsystems.agitator;

import com.revrobotics.spark.SparkMax;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;

public interface AgitatorIO {
    @AutoLog
    public static class AgitatorIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(AgitatorIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void setPercent(double percent) {}

    public default ArrayList<SparkMax> getSparks() {
        return new ArrayList<>();
    }
}
