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
import java.util.HashSet;
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

    @AutoLogOutput private boolean usePIDF = true;

    public enum Requester {
        AutonomouseSpinUp,
        AutonomouseShoot,
        InAllianceZone,
        SafeShoot,
        ManualSpinUp,
        ManualShoot
    };

    private final HashSet<Requester> requests = new HashSet<>(Requester.values().length);
    private final StringBuilder reqestsStringBuilder = new StringBuilder();

    private final DoubleSupplier distanceToHub;
    @AutoLogOutput private DoubleSupplier speedOverride = null;

    public Shooter(ShooterIO io, DoubleSupplier distanceToHub) {
        this.io = io;
        this.distanceToHub = distanceToHub;

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
                                (voltage) -> {
                                    usePIDF = false;
                                    io.setVoltage(voltage.baseUnitMagnitude());
                                },
                                null,
                                this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Info/DistanceToHub", distanceToHub);

        reqestsStringBuilder.delete(0, reqestsStringBuilder.length());
        requests.forEach(
                (req) -> {
                    reqestsStringBuilder.append(req.toString());
                    reqestsStringBuilder.append(',');
                });
        Logger.recordOutput("Shooter/Requests", reqestsStringBuilder.toString());

        if (usePIDF) {
            if (currentlyRequested() || speedOverride != null) {
                pid.setSetpoint(
                        speedOverride == null
                                ? calculateSpeed(distanceToHub.getAsDouble())
                                : speedOverride.getAsDouble());
                io.setVoltage(feedforward.calculate(pid.getSetpoint()) + pid.calculate(inputs.velocityRPM));
                Logger.recordOutput("Shooter/Setpoint RPM", pid.getSetpoint());
                Logger.recordOutput("Shooter/Error RPM", pid.getError());
                Logger.recordOutput("Shooter/Error Squared", pid.getError() * pid.getError());
            } else {
                io.setVoltage(0);
                pid.reset();
            }
        }
    }

    @AutoLogOutput
    public boolean currentlyRequested() {
        return requests.size() > 0;
    }

    /**
     * Add a request to hold the shooter at speed.
     *
     * @param requester where the request is coming from
     * @return a StartEnd command that adds the request at start and removes the request at end.
     */
    public Command requestSpinUp(Requester requester) {
        return Commands.startEnd(() -> requests.add(requester), () -> requests.remove(requester));
    }

    /**
     * Spin up the shooter blind to a supplied speed.
     *
     * @param speedSupplier supplies the speed in RPM to spin the shooter
     * @return a StartEnd command that spins to the custom speed and then releases the shooter at the
     *     end.
     */
    public Command spinUpBlind(DoubleSupplier speedSupplier) {
        return Commands.startEnd(() -> speedOverride = speedSupplier, () -> speedOverride = null);
    }

    /**
     * Wait until the shooter is at it's overriden, blind speed setpoint
     *
     * @return a Wait command
     */
    public Command waitForBlindSetpoint() {
        return Commands.waitUntil(this::atBlindSetpoint);
    }

    @AutoLogOutput
    public boolean atBlindSetpoint() {
        return pid.atSetpoint() && speedOverride != null && usePIDF;
    }

    /**
     * Waits for the shooter to be at its setpoint.
     *
     * @return a Wait command that finishes when all of the following conditions are met:
     *     <ol>
     *       <li>the PID is at its setpoint
     *       <li>the shooter is currently under request to be spun up to speed
     *       <li>the PIDF is enabled (disabled during sysid)
     *     </ol>
     */
    public Command waitForSetpoint() {
        return Commands.waitUntil(this::atSetpoint);
    }

    /**
     * @return true under the following conditions:
     *     <ol>
     *       <li>the PID is at its setpoint
     *       <li>the shooter is currently under request to be spun up to speed
     *       <li>the PIDF is enabled (disabled during sysid)
     *     </ol>
     */
    @AutoLogOutput
    public boolean atSetpoint() {
        return pid.atSetpoint() && currentlyRequested() && usePIDF;
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
        final double b = 360d; // linear term
        final double c = 1741d; // y intercept
        return distance * distance * a + distance * b + c;
    }

    private Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysID.quasistatic(direction);
    }

    private Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysID.dynamic(direction);
    }

    public Command allSysIds() {
        final double pauseBetweenRoutines = 3.0;
        return Commands.sequence(
                enablePIDF(false),
                sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(pauseBetweenRoutines),
                sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(pauseBetweenRoutines),
                sysIdDynamic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(pauseBetweenRoutines),
                sysIdDynamic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(pauseBetweenRoutines),
                enablePIDF(true));
    }

    private Command enablePIDF(boolean enable) {
        return Commands.runOnce(() -> usePIDF = enable, this);
    }

    @Override
    public ArrayList<SparkMax> getSparks() {
        return io.getSparks();
    }

    public double getRotationRPM() {
        return inputs.velocityRPM;
    }

    public Command setLinearPosition(DoubleSupplier position) {
        return Commands.run(() -> io.setLinearPosition(position.getAsDouble()));
    }
}
