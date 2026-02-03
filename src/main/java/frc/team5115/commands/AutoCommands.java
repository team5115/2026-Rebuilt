package frc.team5115.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5115.subsystems.shooter.Shooter;
import frc.team5115.subsystems.agitator.Agitator;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.indexer.Indexer;
import frc.team5115.subsystems.intake.Intake;
import frc.team5115.Constants.AutoConstants;


public class AutoCommands {
    private AutoCommands() {}

    // TODO auto commands!!!

    //verify this works with pathplanner later 
    public static Command Intake(Intake intake, Agitator agitator){
        return Commands.parallel(
            intake.intake(),
            agitator.slow()
        );
    }

    public static Command Shoot(Drivetrain drivetrain, Agitator agitator, Indexer indexer, Shooter shooter){ 
         return Commands.parallel(
                agitator.fast(),
                shooter.maintainSpeed(() -> AutoConstants.distanceToHub(drivetrain.getPose())),
                shooter.waitForSetpoint().raceWith(indexer.reject()).andThen(indexer.index())); 
    }
}
