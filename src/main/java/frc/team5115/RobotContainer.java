package frc.team5115;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.team5115.Constants.Mode;
import frc.team5115.subsystems.agitator.Agitator;
import frc.team5115.subsystems.agitator.AgitatorIOSparkMax;
import frc.team5115.subsystems.bling.Bling;
import frc.team5115.subsystems.bling.BlingIO;
import frc.team5115.subsystems.bling.BlingIOReal;
import frc.team5115.subsystems.bling.BlingIOSim;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.drive.GyroIO;
import frc.team5115.subsystems.drive.GyroIONavx;
import frc.team5115.subsystems.drive.GyroIOSim;
import frc.team5115.subsystems.drive.ModuleIO;
import frc.team5115.subsystems.drive.ModuleIOSim;
import frc.team5115.subsystems.drive.ModuleIOSparkMax;
import frc.team5115.subsystems.indexer.Indexer;
import frc.team5115.subsystems.indexer.IndexerIO;
import frc.team5115.subsystems.indexer.IndexerIOSim;
import frc.team5115.subsystems.indexer.IndexerIOSparkMax;
import frc.team5115.subsystems.intake.Intake;
import frc.team5115.subsystems.intake.IntakeIO;
import frc.team5115.subsystems.intake.IntakeIOSim;
import frc.team5115.subsystems.intake.IntakeIOSparkMax;
import frc.team5115.subsystems.shooter.Shooter;
import frc.team5115.subsystems.shooter.ShooterIO;
import frc.team5115.subsystems.shooter.ShooterIOSim;
import frc.team5115.subsystems.shooter.ShooterIOSparkMax;
import frc.team5115.subsystems.vision.PhotonVision;
import frc.team5115.subsystems.vision.PhotonVisionIO;
import frc.team5115.subsystems.vision.PhotonVisionIOReal;
import frc.team5115.subsystems.vision.PhotonVisionIOSim;
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
    private final PhotonVision vision;
    private final Intake intake;
    private final Bling bling;
    private final Shooter shooter;
    private final Indexer indexer;
    private final Agitator agitator;

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

        switch (Constants.currentMode) {
            case REAL:
                gyro = new GyroIONavx();
                intake = new Intake(new IntakeIOSparkMax());
                drivetrain =
                        new Drivetrain(
                                gyro,
                                new ModuleIOSparkMax(0),
                                new ModuleIOSparkMax(1),
                                new ModuleIOSparkMax(2),
                                new ModuleIOSparkMax(3),
                                (pose) -> {});
                vision = new PhotonVision(new PhotonVisionIOReal(), drivetrain);
                bling = new Bling(new BlingIOReal());
                shooter = new Shooter(new ShooterIOSparkMax());
                indexer = new Indexer(new IndexerIOSparkMax());
                agitator = new Agitator(new AgitatorIOSparkMax());
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                MapleSim.getInstance();
                MapleSim.setupArena();
                MapleSim.initInstance();
                var swerveSim = MapleSim.getSwerveSim();
                gyro = new GyroIOSim(swerveSim.getGyroSimulation());

                intake = new Intake(new IntakeIOSim());
                drivetrain =
                        new Drivetrain(
                                gyro,
                                new ModuleIOSim(swerveSim.getModules()[0]),
                                new ModuleIOSim(swerveSim.getModules()[1]),
                                new ModuleIOSim(swerveSim.getModules()[2]),
                                new ModuleIOSim(swerveSim.getModules()[3]),
                                swerveSim::setSimulationWorldPose);
                vision = new PhotonVision(new PhotonVisionIOSim(), drivetrain);
                bling = new Bling(new BlingIOSim());
                shooter = new Shooter(new ShooterIOSim());
                indexer = new Indexer(new IndexerIOSim());
                agitator = new Agitator(new AgitatorIOSparkMax());
                break;

            default:
                // Replayed robot, disable IO implementations
                gyro = new GyroIO() {};
                intake = new Intake(new IntakeIO() {});
                drivetrain =
                        new Drivetrain(
                                gyro,
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                (pose) -> {});
                vision = new PhotonVision(new PhotonVisionIO() {}, drivetrain);
                bling = new Bling(new BlingIO() {});
                shooter = new Shooter(new ShooterIO() {});
                indexer = new Indexer(new IndexerIO() {});
                agitator = new Agitator(new AgitatorIOSparkMax());
                break;
        }
        driverController = new DriverController();

        // Register auto commands for pathplanner
        registerCommands(drivetrain, vision, intake, shooter, indexer);

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

        autoChooser.addOption(
                "Shooter SysID (Quasistatic Forward)", shooter.sysIdQuasistatic(Direction.kForward));
        autoChooser.addOption(
                "Shooter SysID (Quasistatic Reverse)", shooter.sysIdQuasistatic(Direction.kReverse));
        autoChooser.addOption(
                "Shooter SysID (Dynamic Forward)", shooter.sysIdDynamic(Direction.kForward));
        autoChooser.addOption(
                "Shooter SysID (Dynamic Reverse)", shooter.sysIdDynamic(Direction.kReverse));

        autoChooser.addOption("Shooter All SysIds", shooter.allSysIds());

        driverController.configureButtonBindings(drivetrain, intake, agitator, indexer, shooter);
        driverController.configureRumbleBindings(drivetrain);
        configureBlingBindings();
    }

    private void configureBlingBindings() {
        bling.setDefaultCommand(bling.redKITT().ignoringDisable(true));
        drivetrain.aligningToGoal().whileTrue(bling.yellowScrollIn());
        drivetrain.alignedAtGoalTrigger().whileTrue(bling.whiteScrollIn());
        new Trigger(() -> hasFaults).whileTrue(bling.faultFlash().ignoringDisable(true));
    }

    public void robotPeriodic() {
        if (Constants.currentMode == Mode.REAL) {
            if (faultPrintTimeout <= 0) {
                final var faults =
                        RobotFaults.fromSubsystems(
                                drivetrain,
                                vision,
                                intake,
                                shooter,
                                indexer,
                                driverController.joysticksConnected());
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
    public static void registerCommands(
            Drivetrain drivetrain, PhotonVision vision, Intake intake, Shooter shooter, Indexer indexer) {

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
    }

    public void simInit() {
        drivetrain.setPose(Constants.SIM_INIT_POSE);
    }

    public void simPeriodic() {}

    public void autoInit() {
        drivetrain.setAutoCurrentLimit();
        // Offset gyro to zero
        drivetrain.offsetGyro();
        // Then offset by 180 degrees
        drivetrain.offsetGyro(Rotation2d.k180deg);
    }

    public void disabledPeriodic() {}
}
