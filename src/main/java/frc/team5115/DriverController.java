package frc.team5115;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team5115.Constants.AutoConstants.Side;
import frc.team5115.commands.DriveCommands;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.intake.Intake;

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

    public void configureButtonBindings(Drivetrain drivetrain, Intake intake) {
        // drive control
        drivetrain.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drivetrain,
                        () -> robotRelative,
                        () -> slowMode,
                        () -> -joyDrive.getLeftY(),
                        () -> -joyDrive.getLeftX(),
                        () -> -joyDrive.getRightX()));
        if (Constants.SINGLE_MODE) {
            configureSingleMode(drivetrain, intake);
        } else {
            configureDualMode(drivetrain, intake);
        }
    }

    private void configureSingleMode(Drivetrain drivetrain, Intake intake) {
        /* Drive button bindings -
         * x: forces the robot to stop moving
         * left bumper: Sets robot relative to true while held down
         * right bumper: Sets slow mode while held down
         * left and right triggers align to score respectively
         * both triggers aligns to the middle
         * start resets field orientation
         */

        // joyDrive.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        // joyDrive.leftBumper().onTrue(setRobotRelative(true)).onFalse(setRobotRelative(false));
        // joyDrive.rightBumper().onTrue(setSlowMode(true)).onFalse(setSlowMode(false));

        joyDrive.start().onTrue(offsetGyro(drivetrain));

        joyDrive
                .leftTrigger()
                .and(joyDrive.rightTrigger().negate())
                .onTrue(drivetrain.selectNearestScoringSpot(Side.LEFT))
                .whileTrue(drivetrain.alignSelectedSpot());
        joyDrive
                .rightTrigger()
                .and(joyDrive.leftTrigger().negate())
                .onTrue(drivetrain.selectNearestScoringSpot(Side.RIGHT))
                .whileTrue(drivetrain.alignSelectedSpot());

        joyDrive
                .leftTrigger()
                .and(joyDrive.rightTrigger())
                .onTrue(drivetrain.selectNearestScoringSpot(Side.CENTER))
                .whileTrue(drivetrain.alignSelectedSpot());

        /*
        * Manipulator button bindings:
        * hold left stick and move it for elevator manual control
        * hold start for L1
        * hold b for L2
        * hold x for L3
        * press back to rezero elevator
        * hold a to vomit
        * hold right trigger to dispense
        * hold left trigger to reverse dispense
        * press right bumper to extend climb piston
        * press left bumper to retract climb piston
        * point up on dpad to toggle climber block
        //  * point down on dpad and press B (L2) or X (L3) to clean algae, release to stow
        */

        // TODO add binds
    }

    private void configureDualMode(Drivetrain drivetrain, Intake intake) {
        /* Drive button bindings -
         * x: forces the robot to stop moving
         * left bumper: Sets robot relative to true while held down
         * right bumper: Sets slow mode while held down
         * left and right triggers align to score respectively
         * both triggers aligns to the middle
         * start resets field orientation
         */

        joyDrive.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        joyDrive.leftBumper().onTrue(setRobotRelative(true)).onFalse(setRobotRelative(false));
        joyDrive.rightBumper().onTrue(setSlowMode(true)).onFalse(setSlowMode(false));
        joyDrive.start().onTrue(offsetGyro(drivetrain));

        joyDrive
                .leftTrigger()
                .and(joyDrive.rightTrigger().negate())
                .onTrue(drivetrain.selectNearestScoringSpot(Side.LEFT))
                .whileTrue(drivetrain.alignSelectedSpot());
        joyDrive
                .rightTrigger()
                .and(joyDrive.leftTrigger().negate())
                .onTrue(drivetrain.selectNearestScoringSpot(Side.RIGHT))
                .whileTrue(drivetrain.alignSelectedSpot());

        joyDrive
                .leftTrigger()
                .and(joyDrive.rightTrigger())
                .onTrue(drivetrain.selectNearestScoringSpot(Side.CENTER))
                .whileTrue(drivetrain.alignSelectedSpot());

        /*
        * Manipulator button bindings:
        * hold left stick and move it for elevator manual control
        * hold start for L1
        * hold b for L2
        * hold x for L3
        * press back to rezero elevator
        * hold a to vomit
        * hold right trigger to dispense
        * hold left trigger to reverse dispense
        * press right bumper to extend climb piston
        * press left bumper to retract climb piston
        * point up on dpad to toggle climber block
        //  * point down on dpad and press B (L2) or X (L3) to clean algae, release to stow
        */

        // TODO add binds
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
