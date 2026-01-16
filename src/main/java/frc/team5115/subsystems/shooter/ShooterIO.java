package frc.team5115.subsystems.shooter;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkMax;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {}

    /** Run the shooter motor at the specified voltage. */
    public default void setVoltage(double volts) {}

    public default void getSparks(ArrayList<SparkMax> sparks) {}
}