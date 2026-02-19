package frc.team5115;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants.AutoConstants;
import frc.team5115.commands.AutoCommands;
import frc.team5115.subsystems.agitator.Agitator;
import frc.team5115.subsystems.agitator.AgitatorIOSim;
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
    private final GyroIO gyro;
    private final Drivetrain drivetrain;
    private final PhotonVision vision;
    private final Intake intake;
    private final Bling bling;
    private final Shooter shooter;
    private final Indexer indexer;
    private final Agitator agitator;
    private final RobotFaults faults;

    // Controllers
    private final Bindings bindings;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    private final DoubleSupplier blindSpeedSupplier;
    private final BooleanSupplier hitTargetSupplier;
    private final BooleanConsumer hitTargetConsumer;

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
                shooter = new Shooter(new ShooterIOSparkMax(), drivetrain::getDistanceToHub);
                indexer = new Indexer(new IndexerIOSparkMax());
                agitator = new Agitator(new AgitatorIOSparkMax());
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                MapleSim.initializeArena();
                final var swerveSim = MapleSim.getSwerveSim();
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
                shooter = new Shooter(new ShooterIOSim(), drivetrain::getDistanceToHub);
                indexer = new Indexer(new IndexerIOSim());
                agitator = new Agitator(new AgitatorIOSim());

                MapleSim.initializeTriggers(indexer, shooter);
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
                shooter = new Shooter(new ShooterIO() {}, drivetrain::getDistanceToHub);
                indexer = new Indexer(new IndexerIO() {});
                agitator = new Agitator(new AgitatorIOSparkMax());
                break;
        }

        // Register auto commands for pathplanner
        registerCommands();

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

        autoChooser.addOption("Shooter All SysIds", shooter.allSysIds());

        final String speedKey = "ShooterSpeedInput";
        final String targetKey = "HitTarget?";
        SmartDashboard.putNumber(speedKey, 0);
        SmartDashboard.putBoolean(targetKey, false);
        blindSpeedSupplier = () -> SmartDashboard.getNumber(speedKey, 1000);
        hitTargetSupplier = () -> SmartDashboard.getBoolean(targetKey, false);
        hitTargetConsumer = (v) -> SmartDashboard.putBoolean(targetKey, v);

        // Initialize bindings and robot faults
        bindings = new Bindings(drivetrain, intake, agitator, indexer, shooter);
        faults =
                new RobotFaults(
                        drivetrain, vision, bindings::joysticksConnected, intake, agitator, indexer, shooter);

        bindings.configureButtonBindings(blindSpeedSupplier);
        bindings.configureBlingBindings(bling, faults);
    }

    /** Register commands for pathplanner to use in autos. */
    private void registerCommands() {
        NamedCommands.registerCommand("Intake", AutoCommands.intake(intake, agitator, indexer));
        NamedCommands.registerCommand(
                "Shoot", AutoCommands.shoot(3, drivetrain, agitator, indexer, shooter));
        NamedCommands.registerCommand(
                "Shoot Forever", AutoCommands.shoot(20, drivetrain, agitator, indexer, shooter));
        NamedCommands.registerCommand("Spin Up", AutoCommands.spinUp(agitator, indexer, shooter));
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

    public void robotPeriodic() {
        if (hitTargetSupplier.getAsBoolean()) {
            Logger.recordOutput(
                    "ShooterData/SuccessfulDistance", Meters.of(drivetrain.getDistanceToHub()));
            Logger.recordOutput(
                    "ShooterData/SuccessfulSpeed", RotationsPerSecond.of(blindSpeedSupplier.getAsDouble()));
            hitTargetConsumer.accept(false);
        }

        Logger.recordOutput(
                "SubZone",
                Constants.isRedAlliance() ? AutoConstants.RED_SUB_ZONE : AutoConstants.BLUE_SUB_ZONE);

        Logger.recordOutput(
                "AllianceZone",
                Constants.isRedAlliance()
                        ? AutoConstants.RED_ALLIANCE_ZONE
                        : AutoConstants.BLUE_ALLIANCE_ZONE);

        Logger.recordOutput("Info/IsHubActive?", Constants.isHubActive());
        Logger.recordOutput("Info/IsRedAlliance?", Constants.isRedAlliance());
        Logger.recordOutput("Info/IsSafeToShoot?", bindings.safeToShoot());
        Logger.recordOutput("Info/IsAutomationEnabled?", bindings.automationEnabled());

        faults.periodic();
    }

    public void simPeriodic() {
        MapleSim.simPeriodic(intake, indexer, shooter);
    }

    public void teleopInit() {
        drivetrain.setTeleopCurrentLimit();
    }

    public void autoInit() {
        drivetrain.setAutoCurrentLimit();
        // TODO offset the gyro to compensate for auto starting pose
        // ? How exactly do we do this considering our autos start (and end) in different rotations?
        drivetrain.zeroGyro();
        // drivetrain.zeroGyro(drivetrain.getPose().getRotation());

        if (Constants.currentMode == Constants.Mode.SIM) {
            MapleSim.resetForAuto();
        }
    }
}
