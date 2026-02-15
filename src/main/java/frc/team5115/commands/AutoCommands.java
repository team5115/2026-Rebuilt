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

    // TODO verify auto works
    public static Command intake(Intake intake, Agitator agitator) {
        return Commands.parallel(intake.intake(), agitator.slow());
    }

    public static Command shoot(
            double timeout, Drivetrain drivetrain, Agitator agitator, Indexer indexer, Shooter shooter) {
        return Commands.parallel(
                        agitator.fast(),
                        shooter.requestSpinUp(Shooter.Requester.AutonomousPeriod),
                        shooter.waitForSetpoint().raceWith(indexer.reject()).andThen(indexer.index()))
                .withTimeout(timeout);
    }

    public static Command spinUp(Shooter shooter) {
        return shooter.requestSpinUp(Shooter.Requester.DumbShoot); // TODO fix auto dumb spin up
    }
}
