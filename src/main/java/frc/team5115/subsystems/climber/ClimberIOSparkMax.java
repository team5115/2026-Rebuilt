package frc.team5115.subsystems.climber;

import java.util.ArrayList;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberIOSparkMax implements ClimberIO{
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public ClimberIOSparkMax() {
        motor = new SparkMax(0, MotorType.kBrushless);
        encoder = motor.getEncoder();

        final SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
                .closedLoopRampRate(0.1)
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.velocityRPM = encoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.positionRad = encoder.getPosition() * 2 * Math.PI;
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public ArrayList<SparkMax> getSparks() {
        ArrayList<SparkMax> sparks = new ArrayList<>();
        sparks.add(motor);
        return sparks;
    }
}
