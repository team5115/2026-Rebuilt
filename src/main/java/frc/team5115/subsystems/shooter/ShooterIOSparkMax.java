package frc.team5115.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.PWM;
import frc.team5115.Constants;
import java.util.ArrayList;

public class ShooterIOSparkMax implements ShooterIO {
    private final PWM linearActuator1;
    private final PWM linearActuator2;
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public ShooterIOSparkMax() {
        motor = new SparkMax(Constants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        final SparkMaxConfig motorConfig = new SparkMaxConfig();
        // Shooter motor configs
        motorConfig
                .closedLoopRampRate(0.1)
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        linearActuator1 = new PWM(0);
        linearActuator2 = new PWM(1);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.velocityRPM = encoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.position = encoder.getPosition();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setLinearPosition(double position) {
        linearActuator1.setPosition(position);
        linearActuator2.setPosition(position);
    }

    @Override
    public ArrayList<SparkMax> getSparks() {
        ArrayList<SparkMax> sparks = new ArrayList<>();
        sparks.add(motor);
        return sparks;
    }
}
