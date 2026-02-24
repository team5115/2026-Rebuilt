package frc.team5115.subsystems.climber;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
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

import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase implements MotorContainer {
    final ClimberIO io;
    ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final SimpleMotorFeedforward feedforward;
    private final PIDController pid;
    private SysIdRoutine sysId;

    public Climber(ClimberIO io) {
        this.io = io;

        // TODO update ff and pid values
        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                feedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
                pid = new PIDController(0.0, 0, 0);
                break;
            case SIM:
                feedforward = new SimpleMotorFeedforward(0, 0.0, 0.0);
                pid = new PIDController(0.0, 0, 0);
                break;
            default:
                feedforward = new SimpleMotorFeedforward(0, 0, 0);
                pid = new PIDController(0, 0, 0);
                break;
        }

        pid.setTolerance(0); // TODO edit tolerance

        SmartDashboard.putData("Climber/PIDController", pid);

        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> {
                                    Logger.recordOutput("Climber/SysIdState", state.toString());
                                    Logger.recordOutput("Climber/PositionRadians", inputs.positionRad);
                                    Logger.recordOutput(
                                            "Climber/VelocityRadPerSec",
                                            Units.rotationsPerMinuteToRadiansPerSecond(inputs.velocityRPM));
                                }),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> {
                                    io.setVoltage(voltage.baseUnitMagnitude());
                                },
                                null,
                                this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        double speed = pid.calculate(inputs.positionRad);
        double volts = feedforward.calculate(speed);
        volts = MathUtil.clamp(volts, -12, +12);

        if(volts < 0.1)
            volts = 0;
        io.setVoltage(volts);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
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
                sysIdDynamic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(pauseBetweenRoutines)
            );
    }

    public Command deployClimb() {
        return Commands.run(
                () -> pid.setSetpoint(0), // TODO change deploy position
                this);
    }

    public Command climb() {
        return Commands.run(
                () -> pid.setSetpoint(0), // TODO change climb position
                this);
    }

    public Command waitForClimb() {
        return Commands.waitUntil(pid::atSetpoint);
    }

    public void stop() {
        io.setVoltage(0);
    }

    @Override
    public ArrayList<SparkMax> getSparks() {
        return io.getSparks();
    }
}
