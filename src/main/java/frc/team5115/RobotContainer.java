package frc.team5115;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.team5115.Constants.AutoConstants;
import frc.team5115.subsystems.shooter.Shooter;
import frc.team5115.subsystems.shooter.ShooterIO;
import frc.team5115.subsystems.shooter.ShooterIOSim;
import frc.team5115.subsystems.shooter.ShooterIOSparkMax;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    // private final GyroIO gyro;
    // private final Drivetrain drivetrain;
    private final Shooter shooter;

    // Controllers
    private final DriverController driverController;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    private final DoubleSupplier speedSupplier;
    private final BooleanSupplier hitTargetSupplier;
    private final BooleanConsumer hitTargetConsumer;
    private final DoubleSupplier distanceMetersSupplier;

    // Works with faults

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        AutoConstants.precomputeAlignmentPoses(); // Computes robot starting pose with vision

        switch (Constants.currentMode) {
            case REAL:
                // gyro = new GyroIONavx();
                // drivetrain =
                //         new Drivetrain(
                //                 gyro,
                //                 new ModuleIOSparkMax(0),
                //                 new ModuleIOSparkMax(1),
                //                 new ModuleIOSparkMax(2),
                //                 new ModuleIOSparkMax(3),
                //                 (pose) -> {});
                shooter = new Shooter(new ShooterIOSparkMax());
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                MapleSim.getInstance();
                MapleSim.setupArena();
                MapleSim.initInstance();
                // var swerveSim = MapleSim.getSwerveSim();
                // gyro = new GyroIOSim(swerveSim.getGyroSimulation());

                // drivetrain =
                //         new Drivetrain(
                //                 gyro,
                //                 new ModuleIOSim(swerveSim.getModules()[0]),
                //                 new ModuleIOSim(swerveSim.getModules()[1]),
                //                 new ModuleIOSim(swerveSim.getModules()[2]),
                //                 new ModuleIOSim(swerveSim.getModules()[3]),
                //                 swerveSim::setSimulationWorldPose);
                shooter = new Shooter(new ShooterIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                // gyro = new GyroIO() {};
                // drivetrain =
                //         new Drivetrain(
                //                 gyro,
                //                 new ModuleIO() {},
                //                 new ModuleIO() {},
                //                 new ModuleIO() {},
                //                 new ModuleIO() {},
                //                 (pose) -> {});
                shooter = new Shooter(new ShooterIO() {});
                break;
        }

        final String speedKey = "ShooterSpeedInput";
        final String targetKey = "HitTarget?";
        final String feetKey = "Feet Distance";
        final String inchKey = "Inch Distance";
        SmartDashboard.putNumber(speedKey, 0);
        SmartDashboard.putBoolean(targetKey, false);
        SmartDashboard.putNumber(feetKey, 0);
        SmartDashboard.putNumber(inchKey, 0);
        speedSupplier = () -> SmartDashboard.getNumber(speedKey, 0);
        hitTargetSupplier = () -> SmartDashboard.getBoolean(targetKey, false);
        hitTargetConsumer = (v) -> SmartDashboard.putBoolean(targetKey, v);
        distanceMetersSupplier =
                () ->
                        Units.feetToMeters(SmartDashboard.getNumber(feetKey, 0))
                                + Units.inchesToMeters(SmartDashboard.getNumber(inchKey, 0));

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // autoChooser.addOption("Drive All SysIds", drivetrain.driveAllSysIds());

        autoChooser.addOption(
                "Shooter SysID (Quasistatic Forward)", shooter.sysIdQuasistatic(Direction.kForward));
        autoChooser.addOption(
                "Shooter SysID (Quasistatic Reverse)", shooter.sysIdQuasistatic(Direction.kReverse));
        autoChooser.addOption(
                "Shooter SysID (Dynamic Forward)", shooter.sysIdDynamic(Direction.kForward));
        autoChooser.addOption(
                "Shooter SysID (Dynamic Reverse)", shooter.sysIdDynamic(Direction.kReverse));

        autoChooser.addOption("Shooter All SysIds", shooter.allSysIds());

        driverController = new DriverController();
        driverController.configureButtonBindings(null, shooter, speedSupplier);
        // driverController.configureRumbleBindings(drivetrain);
    }

    public void robotPeriodic() {
        if (hitTargetSupplier.getAsBoolean()) {
            Logger.recordOutput(
                    "ShooterData/SuccessfulDistance", Meters.of(distanceMetersSupplier.getAsDouble()));
            hitTargetConsumer.accept(false);
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void teleopInit() {
        // drivetrain.setTeleopCurrentLimit();
    }

    public void simInit() {
        // drivetrain.setPose(Constants.SIM_INIT_POSE);
    }

    public void simPeriodic() {}

    public void autoInit() {
        // drivetrain.setAutoCurrentLimit();
        // // Offset gyro to zero
        // drivetrain.offsetGyro();
        // // Then offset by 180 degrees
        // drivetrain.offsetGyro(Rotation2d.k180deg);
    }

    public void disabledPeriodic() {}
}
