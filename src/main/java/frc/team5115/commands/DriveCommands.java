package frc.team5115.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5115.Constants.SwerveConstants;
import frc.team5115.subsystems.agitator.Agitator;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.indexer.Indexer;
import frc.team5115.subsystems.intake.Intake;
import frc.team5115.subsystems.shooter.Shooter;
import frc.team5115.subsystems.shooter.SpeedRequest;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommands {
    private static final double DEADBAND = 0.1;

    private static final double LINEAR_N = 9.0;
    private static final double LINEAR_K = 1.0;

    // TODO maybe modify slow mode speed?
    private static final double SLOW_MODE_MULTIPLIER = 0.10;

    private DriveCommands() {}

    /**
     * Runs forever, maintaining shooter speed based on drivetrain odometry. First, spins up the
     * shooter and agitator while rejecting on indexer. After reaching desired shooter speed, it
     * indexes and continues to shoot until interrupted.
     *
     * @param drivetrain
     * @param agitator
     * @param indexer
     * @param shooter
     * @param request the source of the request for shooting
     * @return a Command that runs forever.
     */
    public static Command smartShoot(
            Drivetrain drivetrain,
            Intake intake,
            Agitator agitator,
            Indexer indexer,
            Shooter shooter,
            SpeedRequest request) {
        return Commands.parallel(
                agitator.fast(),
                intake.intake(),
                shooter.requestSpinUp(request),
                drivetrain.limitCurrent(false),
                shooter.waitForSetpoint().raceWith(indexer.reject()).andThen(indexer.index()));
    }

    public static Command blindShoot(
            Drivetrain drivetrain,
            Intake intake,
            Agitator agitator,
            Indexer indexer,
            Shooter shooter,
            DoubleSupplier shooterSpeed) {
        return Commands.parallel(
                agitator.fast(),
                intake.intake(),
                shooter.spinUpBlind(shooterSpeed),
                drivetrain.limitCurrent(false),
                shooter.waitForBlindSetpoint().raceWith(indexer.reject()).andThen(indexer.index()));
    }

    public static Command spinUp(SpeedRequest request, Drivetrain drivetrain, Shooter shooter) {
        return shooter.requestSpinUp(request).alongWith(drivetrain.limitCurrent(false));
    }

    public static Command vomit(Agitator agitator, Indexer indexer, Intake intake) {
        return Commands.parallel(agitator.vomit(), indexer.vomit(), intake.vomit());
    }

    /**
     * Drive field relative while:
     *
     * <ol>
     *   <li>maintaining a heading pointed towards the hub and
     *   <li>maintaining required shooter speed
     * </ol>
     */
    public static Command lockedOnHub(
            Shooter shooter,
            Drivetrain drivetrain,
            BooleanSupplier slowMode,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
        return Commands.startRun(
                drivetrain::resetAngularPID,
                () -> {
                    final LinearVelocity v = calculateLinearVelocity(slowMode, xSupplier, ySupplier);
                    drivetrain.orbitHub(v.x, v.y);
                },
                drivetrain);
    }

    public static Command fieldRelativeHeadingDrive(
            Drivetrain drivetrain,
            BooleanSupplier slowMode,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier xAngleSupplier,
            DoubleSupplier yAngleSupplier) {
        final var defaultConstraints =
                new TrapezoidProfile.Constraints(
                        SwerveConstants.MAX_ANGULAR_SPEED, SwerveConstants.MAX_ANGULAR_SPEED * 2);
        final var slowConstraints =
                new TrapezoidProfile.Constraints(
                        SwerveConstants.MAX_ANGULAR_SPEED * SLOW_MODE_MULTIPLIER,
                        SwerveConstants.MAX_ANGULAR_SPEED * 2 * SLOW_MODE_MULTIPLIER);
        final var anglePid =
                new ProfiledPIDController(
                        SwerveConstants.ANGULAR_PID_CONSTANTS.kP,
                        SwerveConstants.ANGULAR_PID_CONSTANTS.kI,
                        SwerveConstants.ANGULAR_PID_CONSTANTS.kD,
                        defaultConstraints);
        anglePid.setTolerance(SwerveConstants.ANGULAR_PID_TOLERANCE);
        anglePid.enableContinuousInput(-Math.PI, Math.PI);

        // prevents changing constraints constantly
        final BooleanClass previousSlowMode = new BooleanClass(false);

        return Commands.startRun(
                () -> {
                    final double rotation = drivetrain.getGyroRotation().getRadians();
                    anglePid.reset(rotation);
                    anglePid.setGoal(rotation);
                },
                () -> {
                    if (slowMode.getAsBoolean() != previousSlowMode.b) {
                        previousSlowMode.b = slowMode.getAsBoolean();
                        anglePid.setConstraints(previousSlowMode.b ? slowConstraints : defaultConstraints);
                    }
                    final double angleX = MathUtil.applyDeadband(xAngleSupplier.getAsDouble(), DEADBAND);
                    final double angleY = MathUtil.applyDeadband(yAngleSupplier.getAsDouble(), DEADBAND);
                    final Rotation2d heading =
                            angleX == 0 && angleY == 0
                                    ? drivetrain.getGyroRotation()
                                    : new Rotation2d(angleX, angleY);
                    final double omega =
                            anglePid.calculate(drivetrain.getGyroRotation().getRadians(), heading.getRadians());

                    final LinearVelocity v = calculateLinearVelocity(slowMode, xSupplier, ySupplier);

                    drivetrain.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(v.x, v.y, omega, drivetrain.getGyroRotation()),
                            false,
                            true);
                },
                drivetrain);
    }

    /**
     * Field or robot relative drive command using two joysticks (controlling linear and angular
     * velocities).
     */
    public static Command joystickDrive(
            Drivetrain drivetrain,
            BooleanSupplier robotRelative,
            BooleanSupplier slowMode,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    omega = Math.copySign(responseCurve(omega, 3.0, 0.32), omega);
                    final LinearVelocity v = calculateLinearVelocity(slowMode, xSupplier, ySupplier);

                    omega *=
                            SwerveConstants.MAX_ANGULAR_SPEED
                                    * (slowMode.getAsBoolean() ? SLOW_MODE_MULTIPLIER : 1.0);
                    drivetrain.runVelocity(
                            robotRelative.getAsBoolean()
                                    ? new ChassisSpeeds(v.x, v.y, omega)
                                    : ChassisSpeeds.fromFieldRelativeSpeeds(
                                            v.x, v.y, omega, drivetrain.getGyroRotation()),
                            false,
                            true);
                },
                drivetrain);
    }

    private static Rotation2d calculateLinearDirection(double x, double y) {
        if (x == 0 && y == 0) {
            return new Rotation2d();
        }
        return new Rotation2d(x, y);
    }

    private static double calculateLinearMagnitude(double x, double y) {
        final double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        return responseCurve(linearMagnitude, LINEAR_N, LINEAR_K);
    }

    private static LinearVelocity calculateLinearVelocity(
            BooleanSupplier slowMode, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        final double x = xSupplier.getAsDouble();
        final double y = ySupplier.getAsDouble();
        final Translation2d linearVelocity =
                new Pose2d(new Translation2d(), calculateLinearDirection(x, y))
                        .transformBy(new Transform2d(calculateLinearMagnitude(x, y), 0.0, new Rotation2d()))
                        .getTranslation();
        final double multiplier = slowMode.getAsBoolean() ? SLOW_MODE_MULTIPLIER : 1.0;
        final double vx = linearVelocity.getX() * SwerveConstants.MAX_LINEAR_SPEED * multiplier;
        final double vy = linearVelocity.getY() * SwerveConstants.MAX_LINEAR_SPEED * multiplier;
        return new LinearVelocity(vx, vy);
    }

    /**
     * @param x input value [0,1]
     * @param n power [0,infinity)
     * @param k offset [0,infinity)
     * @return the controller responce curve [0,1]
     */
    private static double responseCurve(double x, double n, double k) {
        x = Math.abs(MathUtil.clamp(x, -1, +1));
        return (Math.pow(x + k, n) + (x - 1) * Math.pow(k, n)) / Math.pow(1 + k, n);
    }

    private static class LinearVelocity {
        public final double x;
        public final double y;

        LinearVelocity(double vx, double vy) {
            x = vx;
            y = vy;
        }
    }

    private static class BooleanClass {
        public boolean b;

        BooleanClass(boolean b) {
            this.b = b;
        }
    }
}

// meow meow meow meooooowwwwwww :3
