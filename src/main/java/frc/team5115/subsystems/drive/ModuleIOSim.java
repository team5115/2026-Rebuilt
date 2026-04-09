package frc.team5115.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team5115.Constants.SwerveConstants;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
    private final SimulatedMotorController.GenericMotorController driveSim;
    private final SimulatedMotorController.GenericMotorController turnSim;
    private final PIDController currentLoop;

    private final SwerveModuleSimulation moduleSimulation;

    // private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 *
    // Math.PI);
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
        driveSim =
                moduleSimulation
                        .useGenericMotorControllerForDrive()
                        .withCurrentLimit(Amps.of(SwerveConstants.DriveMotorCurrentLimit.autoLimit));
        turnSim =
                moduleSimulation
                        .useGenericControllerForSteer()
                        .withCurrentLimit(Amps.of(SwerveConstants.TurningMotorCurrentLimit));
        currentLoop =
                new PIDController(SwerveConstants.CURRENT_LOOP_kP, SwerveConstants.CURRENT_LOOP_kI, 0);
    }

    @Override
    public void setDriveCurrent(double amps) {
        final double measurement = moduleSimulation.getDriveMotorStatorCurrent().in(Amps);
        final double volts = currentLoop.calculate(measurement, amps);
        setDriveVoltage(volts);
        Logger.recordOutput("Test/volts", volts);
        Logger.recordOutput("Test/measurement", measurement);
        Logger.recordOutput("Test/amps", amps);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSim.requestVoltage(Volts.of(volts));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // driveSim.update(Constants.LOOP_PERIOD_SECS);
        // turnSim.update(Constants.LOOP_PERIOD_SECS);

        inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
        inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

        inputs.turnAbsolutePosition = new Rotation2d(moduleSimulation.getSteerAbsoluteAngle());
        inputs.turnVelocityRadPerSec =
                moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.requestVoltage(Volts.of(turnAppliedVolts));
    }
}
