package frc.team5115.subsystems.agitator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase {
    public Agitator() {
        // TODO take AgitatorIO as an argument
    }

    @Override
    public void periodic() {
        // TODO
    }

    /**
     * Run slow forever
     *
     * @return a Run Command
     */
    public Command slow() {
        return run(0.2);
    }

    /**
     * Run fast forever
     *
     * @return a Run Command
     */
    public Command fast() {
        return run(1.0);
    }

    private Command run(double speed) {
        return Commands.run(
                () -> {
                    // TODO
                },
                this);
    }
}
