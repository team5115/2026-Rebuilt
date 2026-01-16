package frc.team5115;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants.AutoConstants;
import frc.team5115.Constants.Mode;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.drive.GyroIO;
import frc.team5115.subsystems.drive.GyroIONavx;
import frc.team5115.subsystems.drive.GyroIOSim;
import frc.team5115.subsystems.drive.ModuleIO;
import frc.team5115.subsystems.drive.ModuleIOSim;
import frc.team5115.subsystems.drive.ModuleIOSparkMax;
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
    private final GyroIO gyro;
    private final Drivetrain drivetrain;

    // Controllers
    private final DriverController driverController;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    // Setings

    private boolean hasFaults = true;
    private double faultPrintTimeout = 0;

    // Works with faults

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        AutoConstants.precomputeAlignmentPoses(); // Computes robot starting pose with vision

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // final GenericEntry dispenseSpeedEntry =
                //         Shuffleboard.getTab("SmartDashboard").add("Dispenser Speed", 0).getEntry();
                //         () -> dispenseSpeedEntry.getDouble(0.1)
                gyro = new GyroIONavx();
                drivetrain =
                        new Drivetrain(
                                gyro,
                                new ModuleIOSparkMax(0),
                                new ModuleIOSparkMax(1),
                                new ModuleIOSparkMax(2),
                                new ModuleIOSparkMax(3),
                                (pose) -> {});
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                MapleSim.getInstance();
                MapleSim.setupArena();
                MapleSim.initInstance();
                var swerveSim = MapleSim.getSwerveSim();
                gyro = new GyroIOSim(swerveSim.getGyroSimulation());

                drivetrain =
                        new Drivetrain(
                                gyro,
                                new ModuleIOSim(swerveSim.getModules()[0]),
                                new ModuleIOSim(swerveSim.getModules()[1]),
                                new ModuleIOSim(swerveSim.getModules()[3]),
                                new ModuleIOSim(swerveSim.getModules()[2]),
                                swerveSim::setSimulationWorldPose);
                break;

            default:
                // Replayed robot, disable IO implementations
                gyro = new GyroIO() {};
                drivetrain =
                        new Drivetrain(
                                gyro,
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                (pose) -> {});
                break;
        }
        driverController = new DriverController();

        // Register auto commands for pathplanner
        registerCommands(drivetrain);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        autoChooser.addOption(
                "Drive Spin SysId (Quasistatic Forward)",
                drivetrain.sysIdSpinQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive Spin SysId (Quasistatic Reverse)",
                drivetrain.sysIdSpinQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive Spin SysId (Dynamic Forward)",
                drivetrain.sysIdSpinDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive Spin SysId (Dynamic Reverse)",
                drivetrain.sysIdSpinDynamic(SysIdRoutine.Direction.kReverse));

        autoChooser.addOption("Drive All SysIds", drivetrain.driveAllSysIds());

        driverController.configureButtonBindings(drivetrain);
        driverController.configureRumbleBindings(drivetrain);
    }

    public void robotPeriodic() {
        if (Constants.currentMode == Mode.REAL) {
            if (faultPrintTimeout <= 0) {
                final var faults =
                        RobotFaults.fromSubsystems(drivetrain, driverController.joysticksConnected());
                hasFaults = faults.hasFaults();
                if (hasFaults) {
                    System.err.println(faults.toString());
                }
                faultPrintTimeout = 50;
            }
            faultPrintTimeout -= 1;
            Logger.recordOutput("HasFaults", hasFaults);
            Logger.recordOutput("ClearForMatch", !hasFaults);
        } else {
            MapleSim.simPeriodic();
        }
    }

    /**
     * Register commands for pathplanner to use in autos
     *
     * @param drivetrain
     * @param vision
     * @param intake
     * @param climber
     * @param shooter
     */
    public static void registerCommands(Drivetrain drivetrain) {

        // TODO add named commands
        System.out.println("Registered Commands");
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
        drivetrain.setTeleopCurrentLimit();
        // drivetrain.offsetGyro(fRotation2d.fromDegrees(-90));
    }

    public void autoInit() {
        drivetrain.setAutoCurrentLimit();
        // Offset gyro to zero
        drivetrain.offsetGyro();
        // Then offset by 180 degrees
        drivetrain.offsetGyro(Rotation2d.k180deg);
    }

    public void disabledPeriodic() {}
}
