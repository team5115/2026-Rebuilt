package frc.team5115.subsystems.agitator;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.team5115.Constants;
import java.util.ArrayList;

public class AgitatorIOSparkMax implements AgitatorIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final double gearing = 1f / 4f;

    public AgitatorIOSparkMax() {
        motor = new SparkMax(Constants.AGITATOR_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        final SparkMaxConfig motorConfig = new SparkMaxConfig();
        // Agitator motor configs
        motorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(30, 40);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(AgitatorIOInputs inputs) {
        inputs.velocityRPM = encoder.getVelocity() * gearing;
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPercent(double percent) {
        motor.set(percent);
    }

    @Override
    public ArrayList<SparkMax> getSparks() {
        ArrayList<SparkMax> sparks = new ArrayList<>();
        sparks.add(motor);
        return sparks;
    }
}
