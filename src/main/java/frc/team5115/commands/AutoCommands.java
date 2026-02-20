package frc.team5115.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5115.subsystems.agitator.Agitator;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.indexer.Indexer;
import frc.team5115.subsystems.intake.Intake;
import frc.team5115.subsystems.shooter.Shooter;

public class AutoCommands {
    private AutoCommands() {}

    /** Intake while agitating and rejecting with indexer */
    public static Command intake(Intake intake, Agitator agitator, Indexer indexer) {
        return Commands.parallel(
                Commands.print("Intaking!"), intake.intake(), agitator.slow(), indexer.reject());
    }

    // TODO make auto shoot end once sensor stops detecting balls coming thru
    public static Command shoot(
            double timeout, Drivetrain drivetrain, Agitator agitator, Indexer indexer, Shooter shooter) {
        return Commands.parallel(
                        Commands.print("Shooting!"),
                        agitator.fast(),
                        shooter.requestSpinUp(Shooter.Requester.AutonomouseShoot),
                        shooter
                                .waitForSetpoint()
                                .raceWith(indexer.reject())
                                .andThen(indexer.index().until(indexer.deBounceIsSensing().negate())))
                .withTimeout(timeout);
    }

    /** Spin up the shooter, reject with indexer, and agitate slowly. */
    public static Command spinUp(Agitator agitator, Indexer indexer, Shooter shooter) {
        return Commands.parallel(
                Commands.print("Spinning Up!"),
                shooter.requestSpinUp(Shooter.Requester.AutonomouseSpinUp),
                indexer.reject(),
                agitator.slow());
    }
}
