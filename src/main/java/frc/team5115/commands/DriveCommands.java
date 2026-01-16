package frc.team5115.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5115.Constants.SwerveConstants;
import frc.team5115.subsystems.drive.Drivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommands {
    private static final double DEADBAND = 0.1;

    private static final double LINEAR_N = 6.0;
    private static final double LINEAR_K = 1.0;

    // TODO maybe modify slow mode speed?
    private static final double SLOW_MODE_MULTIPLIER = 0.15;

    private static final Translation2d BLUE_HUB = new Translation2d(4.63, 4.03);
    private static final Translation2d RED_HUB = new Translation2d(11.92, 4.03);

    private DriveCommands() {}

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
                    // Apply deadband
                    double linearMagnitude =
                            MathUtil.applyDeadband(
                                    Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
                    Rotation2d linearDirection;
                    if (xSupplier.getAsDouble() == 0 && ySupplier.getAsDouble() == 0) {
                        linearDirection = new Rotation2d();
                    } else {
                        linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    }
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square values
                    linearMagnitude = responseCurve(linearMagnitude, LINEAR_N, LINEAR_K);
                    omega = Math.copySign(responseCurve(omega, 3.0, 0.32), omega);

                    // Calcaulate new linear velocity
                    Translation2d linearVelocity =
                            new Pose2d(new Translation2d(), linearDirection)
                                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                    .getTranslation();

                    // Convert to ChassisSpeeds & send command
                    final double multiplier = slowMode.getAsBoolean() ? SLOW_MODE_MULTIPLIER : 1.0;
                    final double vx = linearVelocity.getX() * SwerveConstants.MAX_LINEAR_SPEED * multiplier;
                    final double vy = linearVelocity.getY() * SwerveConstants.MAX_LINEAR_SPEED * multiplier;
                    omega *= SwerveConstants.MAX_ANGULAR_SPEED * multiplier;
                    drivetrain.runVelocity(
                            robotRelative.getAsBoolean()
                                    ? new ChassisSpeeds(vx, vy, omega)
                                    : ChassisSpeeds.fromFieldRelativeSpeeds(
                                            vx, vy, omega, drivetrain.getGyroRotation()));
                },
                drivetrain);
    }

    /**
     * @param x input value [0,1]
     * @param n power [0,infinity)
     * @param k offset [0,infinity)
     * @return the controller responce curve [0,1]
     */
    private static double responseCurve(double x, double n, double k) {
        return (Math.pow(x + k, n) + (x - 1) * Math.pow(k, n)) / Math.pow(1 + k, n);
    }
}

// meow meow meow meooooowwwwwww :3
