package frc.team5115;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5115.Constants.AutoConstants;
import frc.team5115.Constants.SwerveConstants;
import frc.team5115.subsystems.indexer.Indexer;
import frc.team5115.subsystems.intake.Intake;
import frc.team5115.subsystems.shooter.Shooter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class MapleSim {
    private static SwerveDriveSimulation swerveSim;
    private static RebuiltFuelOnFly fuelOnFly;
    private static IntakeSimulation intakeSimulation;

    private static final double shotFrequency = 4;
    private static final double shotCooldown = 1d / shotFrequency;
    private static final Timer shotTimer = new Timer();
    private static Trigger indexing;

    public static void initializeArena() {
        final boolean startRed = false;
        final var initPose =
                startRed ? new Pose2d(15.0, 7.5, Rotation2d.kPi) : new Pose2d(1.5, 0.8, Rotation2d.kZero);
        swerveSim = new SwerveDriveSimulation(generateDriveSimConfig(), initPose);
        intakeSimulation =
                IntakeSimulation.InTheFrameIntake(
                        "Fuel",
                        MapleSim.getSwerveSim(),
                        Meters.of(0.470),
                        IntakeSimulation.IntakeSide.FRONT,
                        25);
        final Arena2026Rebuilt arena = (Arena2026Rebuilt) SimulatedArena.getInstance();
        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(3, 3)));
        arena.addDriveTrainSimulation(swerveSim);
        arena.setEfficiencyMode(true);
    }

    public static void resetForAuto() {
        SimulatedArena.getInstance().resetFieldForAuto();
        intakeSimulation.setGamePiecesCount(8);
    }

    public static void initializeTriggers(Indexer indexer, Shooter shooter) {
        shotTimer.start();
        indexing =
                new Trigger(() -> shooter.getRotationRPM() > 500 && indexer.isIndexing())
                        .debounce(Constants.LOOP_PERIOD_SECS * 2d, DebounceType.kBoth);
    }

    public static void simPeriodic(Intake intake, Indexer indexer, Shooter shooter) {
        SimulatedArena.getInstance().simulationPeriodic();

        if (intake.isIntaking()) {
            intakeSimulation.startIntake();
        } else {
            intakeSimulation.stopIntake();
        }

        if (indexing.getAsBoolean()
                && shotTimer.get() >= shotCooldown
                && intakeSimulation.obtainGamePieceFromIntake()) {
            launchFuel(shooter);
        }

        Logger.recordOutput("FieldSimulation/RobotPosition", swerveSim.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    private static DriveTrainSimulationConfig generateDriveSimConfig() {
        return DriveTrainSimulationConfig.Default()
                .withGyro(() -> new GyroSimulation(0, 0))
                .withSwerveModule(generateSwerveModuleConfig())
                .withCustomModuleTranslations(SwerveConstants.MODULE_TRANSLATIONS)
                .withBumperSize(SwerveConstants.BUMPER_WIDTH_X, SwerveConstants.BUMPER_WIDTH_Y)
                .withRobotMass(AutoConstants.ROBOT_MASS);
    }

    private static SwerveModuleSimulationConfig generateSwerveModuleConfig() {
        return new SwerveModuleSimulationConfig(
                DCMotor.getNEO(1), // Drive motor is a NEO
                DCMotor.getNeo550(1), // Steer motor is a NEO 550
                SwerveConstants.DrivingMotorReduction, // Drive motor gear ratio.
                12, // Steer motor gear ratio.
                Volts.of(0.1), // Drive friction voltage.
                Volts.of(0.1), // Steer friction voltage
                Meters.of(SwerveConstants.WHEEL_RADIUS_METERS), // Wheel radius
                KilogramSquareMeters.of(0.005), // Steer MOI
                1.2); // Wheel COF
    }

    public static void launchFuel(Shooter shooter) {
        shotTimer.reset();
        SimulatedArena.getInstance().addGamePieceProjectile(generateFuel(shooter));
    }

    public static GamePieceProjectile generateFuel(Shooter shooter) {
        fuelOnFly =
                new RebuiltFuelOnFly(
                        swerveSim.getSimulatedDriveTrainPose().getTranslation(),
                        new Translation2d(Units.inchesToMeters(-12.46), 0), // shooter position in robot
                        swerveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        swerveSim
                                .getSimulatedDriveTrainPose()
                                .getRotation(), // shooter angle (same as robot angle)
                        Meters.of(Units.inchesToMeters(17.02)), // height of shooter
                        MetersPerSecond.of(
                                RPM.of(shooter.getRotationRPM()).in(RadiansPerSecond)
                                        * Shooter.FLYWHEEL_RADIUS
                                        * 0.5),
                        Degrees.of(90 - 15)); // TODO exit angle of fuel, from the horizontal

        fuelOnFly.withProjectileTrajectoryDisplayCallBack(
                (pose3ds) ->
                        Logger.recordOutput(
                                "Flywheel/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
                (pose3ds) ->
                        Logger.recordOutput(
                                "Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new)));

        fuelOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();

        return fuelOnFly;
    }

    public static SwerveDriveSimulation getSwerveSim() {
        return swerveSim;
    }
}
