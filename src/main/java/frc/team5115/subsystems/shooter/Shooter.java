package frc.team5115.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants;
import frc.team5115.util.MotorContainer;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase implements MotorContainer {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SimpleMotorFeedforward feedforward;
    private final PIDController pid;
    private final SysIdRoutine sysID;

    private double setpointRPM;

    public Shooter(ShooterIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                feedforward = new SimpleMotorFeedforward(0.17484, 0.00223, 0.00030957);
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

        sysID =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> io.setVoltage(voltage.baseUnitMagnitude()), null, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        final double volts =
                feedforward.calculate(setpointRPM) + pid.calculate(inputs.velocityRPM, setpointRPM);
        io.setVoltage(volts);

        final double error = setpointRPM - inputs.velocityRPM;
        Logger.recordOutput("Shooter/Setpoint RPM", setpointRPM);
        Logger.recordOutput("Shooter/Error RPM", error);
        Logger.recordOutput("Shooter/Error Squared", error * error);
        Logger.recordOutput("Shooter/At Setpoint?", pid.atSetpoint());
    }

    /**
     * Update the setpoint of the shooter
     *
     * @param rpm the setpoint speed
     */
    private void setpoint(double rpm) {
        setpointRPM = rpm;
        pid.setSetpoint(rpm);
    }

    /**
     * Instant Command the shooter to spin based on a supplied speed.
     *
     * @param rpm a DoubleSupplier that gives the speed to spin the shooter
     * @return an Instant Command
     */
    public Command supplySetpoint(DoubleSupplier rpm) {
        return Commands.runOnce(() -> setpoint(rpm.getAsDouble()), this);
    }

    /**
     * Instant Command the shooter to spin at a set speed.
     *
     * @param rpm the speed at which to spin the shooter
     * @return an Instant Command
     */
    public Command setSetpoint(double rpm) {
        return Commands.runOnce(() -> setpoint(rpm), this);
    }

    /**
     * Wait until the pid reaches its setpoint
     *
     * @return a Wait Command
     */
    public Command waitForSetpoint() {
        return Commands.waitUntil(() -> pid.atSetpoint());
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
                    // TODO calculate the required shooter speed
                    setpoint(1000 * distanceToHub.getAsDouble());
                },
                () -> {
                    // Stop at the end
                    setpoint(0);
                },
                this);
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
}
