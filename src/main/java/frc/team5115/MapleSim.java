package frc.team5115;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.team5115.Constants.AutoConstants;
import frc.team5115.Constants.SwerveConstants;
import frc.team5115.subsystems.shooter.Shooter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class MapleSim {
    private static SwerveDriveSimulation swerveSim;
    private static RebuiltFuelOnFly fuelOnFly;

    public static void initializeArena() {
        swerveSim = new SwerveDriveSimulation(generateDriveSimConfig(), Constants.SIM_INIT_POSE);
        final Arena2026Rebuilt arena = (Arena2026Rebuilt) SimulatedArena.getInstance();
        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(3, 3)));
        arena.addDriveTrainSimulation(swerveSim);
        arena.setEfficiencyMode(true);
    }

    public static void resetForAuto() {
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public static void simPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();

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
                .withRobotMass(AutoConstants.ROBOT_MASS); // TODO set robot mass
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

    public static void launchFuel() {
        SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
    }

    public static void fuelConfig(Shooter shooter) {
        fuelOnFly =
                new RebuiltFuelOnFly(
                        swerveSim.getSimulatedDriveTrainPose().getTranslation(),
                        new Translation2d(0.2, 0),
                        swerveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        swerveSim.getSimulatedDriveTrainPose().getRotation(),
                        Meters.of(Units.inchesToMeters(17.02)),
                        MetersPerSecond.of(shooter.getRotationRPM() / 6000 * 20),
                        Radians.of(Math.toRadians(55)));

        fuelOnFly.withProjectileTrajectoryDisplayCallBack(
                (pose3ds) ->
                        Logger.recordOutput(
                                "Flywheel/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
                (pose3ds) ->
                        Logger.recordOutput(
                                "Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new)));

        fuelOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();
    }

    public static SwerveDriveSimulation getSwerveSim() {
        return swerveSim;
    }
}
