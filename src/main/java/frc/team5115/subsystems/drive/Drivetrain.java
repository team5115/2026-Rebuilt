package frc.team5115.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants;
import frc.team5115.Constants.AutoConstants;
import frc.team5115.Constants.SwerveConstants;
import frc.team5115.Constants.VisionConstants;
import frc.team5115.util.LocalADStarAK;
import frc.team5115.util.MotorContainer;
import java.util.ArrayList;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase implements MotorContainer {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final SysIdRoutine spinSysId;
    private final Field2d field = new Field2d();

    // TODO tune drive pids
    private final double linear_kp = 1.9 * AutoConstants.MAX_AUTOALIGN_LINEAR_SPEED;
    private final double linear_ki = 0.125 * AutoConstants.MAX_AUTOALIGN_LINEAR_SPEED;
    private final double linear_kd = 0.5 * AutoConstants.MAX_AUTOALIGN_LINEAR_SPEED;
    private final double angular_kp = 0.5 * SwerveConstants.MAX_ANGULAR_SPEED;
    private final double angular_ki = 0.0 * SwerveConstants.MAX_ANGULAR_SPEED;
    private final double angular_kd = 0.0 * SwerveConstants.MAX_ANGULAR_SPEED;

    private final double linearPidTolerence = 0.04;

    private final SlewRateLimiter slewLimiter =
            new SlewRateLimiter(SwerveConstants.MAX_LINEAR_ACCEL, Double.NEGATIVE_INFINITY, 0.0);

    private final ProfiledPIDController angularPID =
            new ProfiledPIDController(
                    angular_kp,
                    angular_ki,
                    angular_kd,
                    new TrapezoidProfile.Constraints(
                            SwerveConstants.MAX_ANGULAR_SPEED, SwerveConstants.MAX_ANGULAR_SPEED * 2));
    private final PIDController translationPid = new PIDController(linear_kp, linear_ki, linear_kd);

    @AutoLogOutput private boolean orbitting = true;

    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(SwerveConstants.MODULE_TRANSLATIONS);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(
                    kinematics,
                    rawGyroRotation,
                    lastModulePositions,
                    new Pose2d(),
                    VisionConstants.VISION_STD_DEV,
                    VisionConstants.STATE_STD_DEV);

    private final Consumer<Pose2d> resetSimulationPoseCallBack;

    public Drivetrain(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            Consumer<Pose2d> resetSimulationPoseCallBack) {
        this.gyroIO = gyroIO;
        this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        angularPID.enableContinuousInput(-Math.PI, Math.PI);
        translationPid.setTolerance(linearPidTolerence);
        translationPid.setIntegratorRange(
                -AutoConstants.MAX_AUTOALIGN_LINEAR_SPEED * 0.5,
                AutoConstants.MAX_AUTOALIGN_LINEAR_SPEED * 0.5);

        // TODO determine angularPID tolerance
        angularPID.setTolerance(Math.toRadians(4));

        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                () -> getChassisSpeeds(),
                (var speeds, var feedforwards) -> runVelocity(speeds, false, false),
                new PPHolonomicDriveController(
                        new PIDConstants(linear_kp, linear_ki, linear_kd),
                        new PIDConstants(angular_kp, angular_ki, angular_kd)),
                SwerveConstants.getRobotConfig(),
                () -> Constants.isRedAlliance(),
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                    field.getObject("traj").setPoses(activePath);
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });

        // Configure SysId
        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("Drivetrain/StraightSysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> {
                                    Logger.recordOutput("Drivetrain/SysIdVoltage", voltage);
                                    final double linearVel =
                                            Math.hypot(
                                                    getChassisSpeeds().vxMetersPerSecond,
                                                    getChassisSpeeds().vxMetersPerSecond);
                                    double distance = 0;
                                    for (var position : getModulePositions()) {
                                        distance += position.distanceMeters;
                                    }
                                    distance /= 4.0;
                                    Logger.recordOutput("Drivetrain/LinearVelocityMetersPerSecond", linearVel);
                                    Logger.recordOutput("Drivetrain/LinearDistanceMeters", distance);
                                    for (int i = 0; i < 4; i++) {
                                        modules[i].runCharacterization(voltage.baseUnitMagnitude());
                                    }
                                },
                                null,
                                this));

        // Configure spinning sysid to spin ccw
        final Rotation2d[] moduleRotations =
                new Rotation2d[] {
                    Rotation2d.fromDegrees(135), // front left
                    Rotation2d.fromDegrees(45), // front right
                    Rotation2d.fromDegrees(225), // back left
                    Rotation2d.fromDegrees(315) // back right
                };
        spinSysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("Drivetrain/SpinSysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> {
                                    updateAngularLogging();
                                    Logger.recordOutput("Drivetrain/SysIdVoltage", voltage);
                                    for (int i = 0; i < 4; i++) {
                                        modules[i].runCharacterization(voltage.baseUnitMagnitude(), moduleRotations[i]);
                                    }
                                },
                                null,
                                this));

        SmartDashboard.putData(
                "Swerve Drive",
                new Sendable() {
                    @Override
                    public void initSendable(SendableBuilder builder) {
                        builder.setSmartDashboardType("SwerveDrive");

                        builder.addDoubleProperty(
                                "Front Left Angle", () -> modules[0].getAngle().getRadians(), null);
                        builder.addDoubleProperty(
                                "Front Left Velocity", () -> modules[0].getVelocityMetersPerSec(), null);

                        builder.addDoubleProperty(
                                "Front Right Angle", () -> modules[1].getAngle().getRadians(), null);
                        builder.addDoubleProperty(
                                "Front Right Velocity", () -> modules[1].getVelocityMetersPerSec(), null);

                        builder.addDoubleProperty(
                                "Back Left Angle", () -> modules[2].getAngle().getRadians(), null);
                        builder.addDoubleProperty(
                                "Back Left Velocity", () -> modules[2].getVelocityMetersPerSec(), null);

                        builder.addDoubleProperty(
                                "Back Right Angle", () -> modules[3].getAngle().getRadians(), null);
                        builder.addDoubleProperty(
                                "Back Right Velocity", () -> modules[3].getVelocityMetersPerSec(), null);

                        builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
                    }
                });

        SmartDashboard.putData(field);

        SmartDashboard.putData("Drivetrain/AnglePIDController", angularPID);
        SmartDashboard.putData("Drivetrain/LinearPIDController", translationPid);
    }

    Rotation2d previousRotation = null;
    Double previousFPGATime = null;

    public void resetAngularPreviousMeasures() {
        previousRotation = null;
        previousFPGATime = null;
    }

    private void updateAngularLogging() {
        if (previousFPGATime != null && previousRotation != null) {
            final double rotationDelta = getGyroRotation().minus(previousRotation).getRadians();
            final double timeDelta = Timer.getFPGATimestamp() - previousFPGATime;
            Logger.recordOutput("Drivetrain/AngularVelocityRadPerSec", rotationDelta / timeDelta);
            Logger.recordOutput("Drivetrain/RotationRadians", getGyroRotation().getRadians());
        }
        previousFPGATime = Timer.getFPGATimestamp();
        previousRotation = getGyroRotation();
    }

    public Command driveAllSysIds() {
        return Commands.sequence(
                sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                resetBetweenSysIdRoutines(),
                sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                resetBetweenSysIdRoutines(),
                sysIdDynamic(SysIdRoutine.Direction.kForward),
                resetBetweenSysIdRoutines(),
                sysIdDynamic(SysIdRoutine.Direction.kReverse),
                resetBetweenSysIdRoutines(),
                sysIdSpinQuasistatic(SysIdRoutine.Direction.kForward),
                resetBetweenSysIdRoutines(),
                sysIdSpinQuasistatic(SysIdRoutine.Direction.kReverse),
                resetBetweenSysIdRoutines(),
                sysIdSpinDynamic(SysIdRoutine.Direction.kForward),
                resetBetweenSysIdRoutines(),
                sysIdSpinDynamic(SysIdRoutine.Direction.kReverse));
    }

    private Command resetBetweenSysIdRoutines() {
        return Commands.waitSeconds(1.5)
                .andThen(Commands.runOnce(this::resetAngularPreviousMeasures, this));
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drivetrain/Gyro", gyroInputs);
        Logger.recordOutput("Drivetrain/GForce", gyroInputs.xyAcceleration / 9.81);
        for (var module : modules) {
            module.periodic();
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }
        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Read wheel positions and deltas from each module
        SwerveModulePosition[] modulePositions = getModulePositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            moduleDeltas[moduleIndex] =
                    new SwerveModulePosition(
                            modulePositions[moduleIndex].distanceMeters
                                    - lastModulePositions[moduleIndex].distanceMeters,
                            modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }

        // Update gyro angle
        if (gyroInputs.connected) {
            // Use the real gyro angle
            rawGyroRotation = gyroInputs.yawPosition;
        } else {
            // Use the angle delta from the kinematics and module deltas
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        // Apply odometry update
        poseEstimator.update(rawGyroRotation, modulePositions);

        // Update field pose
        field.setRobotPose(getPose());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public double getRadialVelocity() {
        final Translation2d centerOfOrbit = AutoConstants.getHubPosition();
        final ChassisSpeeds chassisSpeeds = getChassisSpeeds();
        final Translation2d linearVelocity =
                new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        final Pose2d pose = getPose();
        // Delta is the vector from the robot to the hub
        final Translation2d delta = centerOfOrbit.minus(pose.getTranslation());
        final Rotation2d baseHeading = delta.getAngle();
        final Rotation2d inverseHeading = baseHeading.unaryMinus();

        // Steps to find signed tangential linear velocity (where linear velocity is v)
        // 1a. Project v into the direction of delta to get v_r (radial velocity)
        // 1b. Rotate v_t by the inverse of the angle of delta so its pointed in the X axis
        // 1c. Note the negative sign b/c dr/dt > 0 means moving away from hub
        final Translation2d radialVelocity =
                delta.times(linearVelocity.dot(delta) / delta.getSquaredNorm());
        final double radialVelocityScalar = -radialVelocity.rotateBy(inverseHeading).getX();
        return radialVelocityScalar;
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     * @param orbitting is this an orbit velocity?
     * @param slew should we limit the acceleration?
     */
    public void runVelocity(ChassisSpeeds speeds, boolean orbitting, boolean slew) {
        this.orbitting = orbitting;

        double slewedLinearVelocity =
                slewLimiter.calculate(
                        Math.sqrt(
                                Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2)));
        double angle = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);
        double slewedX = slewedLinearVelocity * Math.cos(angle);
        double slewedY = slewedLinearVelocity * Math.sin(angle);
        ChassisSpeeds slewedChassisSpeeds =
                new ChassisSpeeds(slewedX, slewedY, speeds.omegaRadiansPerSecond);

        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(slewedChassisSpeeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.MAX_LINEAR_SPEED);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
        }

        // Log setpoint states
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
        Logger.recordOutput("Speeds", speeds);
        Logger.recordOutput("SpeedsSlewed", slewedChassisSpeeds);
        Logger.recordOutput("SpeedsDiscrete", discreteSpeeds);
    }

    public void driveFieldRelativeHeading(double vx, double vy, Rotation2d heading) {
        final double omega = angularPID.calculate(getGyroRotation().getRadians(), heading.getRadians());
        runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getGyroRotation()), false, true);
        Logger.recordOutput("Drivetrain/OmegaField", omega);
        Logger.recordOutput("Drivetrain/GoalHeading", heading);
        Logger.recordOutput("Drivetrain/HeadingError", angularPID.getPositionError());
    }

    /**
     * Drive based on field-relative linear velocities with the heading locked toward a position.
     *
     * @param vx x linear velocity
     * @param vy y linear velocity
     */
    public void orbitHub(double vx, double vy) {
        final Translation2d centerOfOrbit = AutoConstants.getHubPosition();
        // We rotate driver input since normal drive is based on getGyroRotation()
        final Rotation2d inputOffset = getRotation().minus(getGyroRotation());
        final Translation2d linearVelocity = new Translation2d(vx, vy).rotateBy(inputOffset);

        final Pose2d pose = getPose();
        // Delta is the vector from the robot to the hub
        final Translation2d delta = centerOfOrbit.minus(pose.getTranslation());
        final Rotation2d baseHeading = delta.getAngle();
        final Rotation2d inverseHeading = baseHeading.unaryMinus();
        final double radius = delta.getNorm();

        // Steps to find signed tangential linear velocity (where linear velocity is v)
        // 1a. Project v into the direction of delta to get v_r (radial velocity)
        // 1b. Rotate v_t by the inverse of the angle of delta so its pointed in the X axis
        // 1c. Note the negative sign b/c dr/dt > 0 means moving away from hub
        final Translation2d radialVelocity =
                delta.times(linearVelocity.dot(delta) / delta.getSquaredNorm());
        final double radialVelocityScalar = -radialVelocity.rotateBy(inverseHeading).getX();

        // 2a. Subtract v_r from v to get v_t (tangential velocity)
        // 2b. Rotate v_t by the inverse of the angle of delta so its pointed in the Y axis
        // 2c. Note the negative sign to match convention of CCW is positive.
        final Translation2d tangentialVelocity = linearVelocity.minus(radialVelocity);
        final double tangentialVelocityScalar = -tangentialVelocity.rotateBy(inverseHeading).getY();

        // 3a. Use this equation to find rotational velocity in terms of linear velocities
        // 3b. Note that we consider radial velocity b/c it affects the radius
        final double orbitOmega =
                tangentialVelocityScalar / radius * (1 - radialVelocityScalar / radius);

        final double pidOmega =
                angularPID.calculate(pose.getRotation().getRadians(), baseHeading.getRadians());
        final double totalOmega = orbitOmega + pidOmega;

        runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX(), linearVelocity.getY(), totalOmega, getRotation()),
                true,
                true);

        Logger.recordOutput("Orbit/OrbitOmega", RadiansPerSecond.of(orbitOmega));
        Logger.recordOutput("Orbit/PidOmega", RadiansPerSecond.of(pidOmega));
        Logger.recordOutput("Orbit/TotalOmega", RadiansPerSecond.of(totalOmega));
        Logger.recordOutput("Orbit/TangentialVelocity", MetersPerSecond.of(tangentialVelocityScalar));
        Logger.recordOutput("Orbit/RadialVelocity", MetersPerSecond.of(radialVelocityScalar));
        Logger.recordOutput("Orbit/TangentialVector", tangentialVelocity.rotateBy(inverseHeading));
        Logger.recordOutput("Orbit/RadialVector", radialVelocity.rotateBy(inverseHeading));
    }

    public void resetAngularPID() {
        angularPID.reset(getRotation().getRadians());
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds(), false, false);
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = SwerveConstants.MODULE_TRANSLATIONS[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    /**
     * Returns a command to run a quasistatic spinning test in the specified direction,forward being
     * ccw
     */
    public Command sysIdSpinQuasistatic(SysIdRoutine.Direction direction) {
        return spinSysId.quasistatic(direction);
    }

    /**
     * Returns a command to run a dynamic test spinning in the specified direction, forward being ccw
     */
    public Command sysIdSpinDynamic(SysIdRoutine.Direction direction) {
        return spinSysId.dynamic(direction);
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public double getDistanceToHub() {
        return AutoConstants.distanceToHub(getPose());
    }

    public boolean lockedOnHub() {
        return angularPID.atSetpoint() && orbitting;
    }

    @AutoLogOutput
    public Trigger inSubZone() {
        return new Trigger(() -> AutoConstants.isInSubZone(getPose()));
    }

    @AutoLogOutput
    public Trigger inAllianceZone() {
        return new Trigger(() -> AutoConstants.isInAllianceZone(getPose()));
    }

    /**
     * Check if the robot is moving within some maximum speed parameters
     *
     * @param maxSpeedMetersPerSec the maximum speed in meters per second of the drivebase
     * @param maxOmegaRadsPerSec the maximum rotational speed in radians per second of the drivebase
     * @return true if the robot is moving below the maximum speed parameters
     */
    public boolean movingWithinTolerance(double maxSpeedMetersPerSec, double maxOmegaRadsPerSec) {
        final var speeds = getChassisSpeeds();
        final double speedSquared =
                speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
                        + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond;

        return Math.abs(speeds.omegaRadiansPerSecond) < maxOmegaRadsPerSec
                && speedSquared < maxSpeedMetersPerSec * maxSpeedMetersPerSec;
    }

    @AutoLogOutput
    public Rotation2d getGyroRotation() {
        if (gyroInputs.connected) {
            return gyroInputs.yawPosition.minus(gyroOffset);
        } else {
            return getRotation();
        }
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        resetSimulationPoseCallBack.accept(pose);
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp The timestamp of the vision measurement in seconds.
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    @AutoLogOutput private Rotation2d gyroOffset = new Rotation2d();

    /** Set the gyro rotation angle to be zero in the current direction. */
    public void zeroGyro() {
        gyroOffset = rawGyroRotation;
    }

    /** Set the gyro rotation angle to be zero in the current direction minus {@code offset}. */
    public void zeroGyro(Rotation2d offset) {
        gyroOffset = rawGyroRotation.minus(offset);
    }

    public ArrayList<SparkMax> getSparks() {
        ArrayList<SparkMax> sparks = new ArrayList<>();
        for (var module : modules) {
            sparks.addAll(module.getAllSparks());
        }
        return sparks;
    }

    private void setDriveCurrentLimits(int amps) {
        for (var module : modules) {
            module.setDriveCurrentLimit(amps);
        }
    }

    /** Set the module drive current limits to the auto current limit */
    public void setTeleopCurrentLimit() {
        setDriveCurrentLimits(SwerveConstants.DrivingMotorTeleopCurrentLimit);
    }

    /** Set the module drive current limits to the auto current limit */
    public void setAutoCurrentLimit() {
        setDriveCurrentLimits(SwerveConstants.DrivingMotorAutoCurrentLimit);
    }

    public boolean isGyroConnected() {
        return gyroInputs.connected;
    }
}
