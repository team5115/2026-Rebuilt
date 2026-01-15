package frc.team5115;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5115.subsystems.intake.Intake;
import frc.team5115.subsystems.intake.IntakeIO;
import frc.team5115.subsystems.intake.IntakeIOSim;
import frc.team5115.subsystems.intake.IntakeIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Intake intake;

    // Controllers
    private final DriverController driverController;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    // Setings

    // Works with faults

    private final DoubleSubscriber speedSubscriber;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // final GenericEntry dispenseSpeedEntry =
                //         Shuffleboard.getTab("SmartDashboard").add("Dispenser Speed", 0).getEntry();
                //         () -> dispenseSpeedEntry.getDouble(0.1)
                final var table = NetworkTableInstance.getDefault().getTable("customtable");
                speedSubscriber = table.getDoubleTopic("speed").subscribe(0);
                intake = new Intake(new IntakeIOSparkMax());
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                intake = new Intake(new IntakeIOSim());
                speedSubscriber = null;
                break;

            default:
                // Replayed robot, disable IO implementations
                intake = new Intake(new IntakeIO() {});
                speedSubscriber = null;
                // TODO set the drivetrain's resetSimulationPoseCallback ^^^
                break;
        }
        driverController = new DriverController();

        // Register auto commands for pathplanner

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        driverController.configureButtonBindings(intake, speedSubscriber::get);
    }

    public void robotPeriodic() {}

    /**
     * Register commands for pathplanner to use in autos
     *
     * @param intake
     */

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void disabledPeriodic() {}

    public void teleopInit() {}

    public void autoInit() {}
}
