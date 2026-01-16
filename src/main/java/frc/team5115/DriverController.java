package frc.team5115;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team5115.commands.DriveCommands;
import frc.team5115.subsystems.drive.Drivetrain;

public class DriverController {
    private final CommandXboxController joyDrive;
    private final CommandXboxController joyManip;

    private boolean robotRelative = false;
    private boolean slowMode = false;

    public DriverController() {
        joyDrive = new CommandXboxController(0);
        joyManip = Constants.SINGLE_MODE ? null : new CommandXboxController(1);
    }

    public boolean joysticksConnected() {
        return joyDrive.isConnected() && (Constants.SINGLE_MODE ? true : joyManip.isConnected());
    }

    private Command offsetGyro(Drivetrain drivetrain) {
        return Commands.runOnce(() -> drivetrain.offsetGyro(), drivetrain).ignoringDisable(true);
    }

    public void configureRumbleBindings(Drivetrain drivetrain) {
        drivetrain.alignedAtGoalTrigger().onTrue(rumble(Constants.RUMBLE_STRENGTH)).onFalse(rumble(0));
    }

    public void configureButtonBindings(Drivetrain drivetrain) {
        // drive control
        drivetrain.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drivetrain,
                        () -> robotRelative,
                        () -> slowMode,
                        () -> -joyDrive.getLeftY(),
                        () -> -joyDrive.getLeftX(),
                        () -> -joyDrive.getRightX()));

        joyDrive.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        joyDrive.leftBumper().onTrue(setRobotRelative(true)).onFalse(setRobotRelative(false));
        joyDrive.rightBumper().onTrue(setSlowMode(true)).onFalse(setSlowMode(false));
        joyDrive.start().onTrue(offsetGyro(drivetrain));
    }

    private Command setRobotRelative(boolean state) {
        return Commands.runOnce(() -> robotRelative = state);
    }

    private Command setSlowMode(boolean state) {
        return Commands.runOnce(() -> slowMode = state);
    }

    public boolean getRobotRelative() {
        return robotRelative;
    }

    public boolean getSlowMode() {
        return slowMode;
    }

    private Command rumble(double value) {
        return Commands.runOnce(
                () -> {
                    joyDrive.setRumble(GenericHID.RumbleType.kBothRumble, value);
                    if (joyManip != null) {
                        joyManip.setRumble(GenericHID.RumbleType.kBothRumble, value);
                    }
                });
    }
}
