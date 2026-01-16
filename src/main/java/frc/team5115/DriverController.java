package frc.team5115;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team5115.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public class DriverController {
    private final CommandXboxController joyDrive;

    private boolean robotRelative = false;
    private boolean slowMode = false;

    public DriverController() {
        joyDrive = new CommandXboxController(0);
    }

    public boolean joysticksConnected() {
        return joyDrive.isConnected();
    }

    public void configureButtonBindings(Intake intake, DoubleSupplier speed) {
        joyDrive.a().whileTrue(intake.setSpeed(speed)).onFalse(intake.stop());
        joyDrive.b().whileTrue(intake.setSpeed(() -> -speed.getAsDouble())).onFalse(intake.stop());
    }

    public boolean getRobotRelative() {
        return robotRelative;
    }

    public boolean getSlowMode() {
        return slowMode;
    }
}
