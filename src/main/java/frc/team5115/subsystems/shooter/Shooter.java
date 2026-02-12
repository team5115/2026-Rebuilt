package frc.team5115.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants;
import frc.team5115.util.MotorContainer;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase implements MotorContainer {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SimpleMotorFeedforward feedforward;
    private final PIDController pid;
    private final SysIdRoutine sysID;

    public static final double FLYWHEEL_MOI = 0.000856915; // kg*m^2
    public static final double FLYWHEEL_RADIUS = Units.inchesToMeters(4.75 / 2);

    private static final double ffConversion = Math.PI / 30;

    @AutoLogOutput private boolean isControlledByPIDF = false;

    public Shooter(ShooterIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                feedforward =
                        new SimpleMotorFeedforward(0.21098, 0.020198 * ffConversion, 0.0046002 * ffConversion);
                pid = new PIDController(4.1686E-05, 0, 0);
                break;
            case SIM:
                feedforward = new SimpleMotorFeedforward(0, 2.10E-3, 0.03);
                pid = new PIDController(0.006, 0, 0);
                break;
            default:
                feedforward = new SimpleMotorFeedforward(0, 0, 0);
                pid = new PIDController(0, 0, 0);
                break;
        }

        pid.setTolerance(20);

        SmartDashboard.putData("Shooter/PIDController", pid);

        sysID =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> {
                                    Logger.recordOutput("Shooter/SysIdState", state.toString());
                                    Logger.recordOutput("Shooter/PositionRadians", inputs.position * 2 * Math.PI);
                                    Logger.recordOutput(
                                            "Shooter/VelocityRadPerSec",
                                            Units.rotationsPerMinuteToRadiansPerSecond(inputs.velocityRPM));
                                }),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> io.setVoltage(voltage.baseUnitMagnitude()), null, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    /**
     * Wait until the PIDF is controlling the shooter and it reaches its setpoint.
     *
     * @return a Wait Command
     */
    public Command waitForSetpoint() {
        return Commands.waitUntil(() -> pid.atSetpoint() && isControlledByPIDF);
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    /**
     * Run forever, maintaining the required shooter speed. Stops the shooter when interrupted.
     *
     * @param distanceToHub the current distance to the hub
     * @return a RunEnd Command
     */
    public Command maintainSpeed(DoubleSupplier distanceToHub) {
        return Commands.runEnd(
                () -> {
                    isControlledByPIDF = true;
                    final double setpoint = calculateSpeed(distanceToHub.getAsDouble());

                    io.setVoltage(
                            feedforward.calculate(setpoint) + pid.calculate(inputs.velocityRPM, setpoint));

                    final double error = pid.getSetpoint() - inputs.velocityRPM;
                    Logger.recordOutput("Shooter/Setpoint RPM", pid.getSetpoint());
                    Logger.recordOutput("Shooter/Error RPM", error);
                    Logger.recordOutput("Shooter/Error Squared", error * error);
                    Logger.recordOutput("Shooter/At Setpoint?", pid.atSetpoint());
                },
                () -> {
                    // Stop at the end
                    pid.setSetpoint(0);
                    io.setVoltage(0);
                    isControlledByPIDF = false;
                },
                this);
    }

    /**
     * Uses an empirically determined best fit function to find the shooter speed based on the
     * distance to the hub.
     *
     * @param distance the distance to the hub along the ground in meters
     * @return the ideal speed in RPM of the shooter
     */
    private static double calculateSpeed(double distance) {
        // TODO determine function for required shooter speed
        final double a = 0d; // squared term
        final double b = 0d; // linear term
        final double c = 500d; // y intercept
        return distance * distance * a + distance * b + c;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysID.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysID.dynamic(direction);
    }

    public Command allSysIds() {
        final double pauseBetweenRoutines = 3.0;
        return Commands.sequence(
                sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(pauseBetweenRoutines),
                sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(pauseBetweenRoutines),
                sysIdDynamic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(pauseBetweenRoutines),
                sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    @Override
    public ArrayList<SparkMax> getSparks() {
        return io.getSparks();
    }

    public double getRotationRPM() {
        return inputs.velocityRPM;
    }
}
