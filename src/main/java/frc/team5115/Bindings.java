package frc.team5115;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5115.commands.DriveCommands;
import frc.team5115.subsystems.agitator.Agitator;
import frc.team5115.subsystems.bling.Bling;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.indexer.Indexer;
import frc.team5115.subsystems.intake.Intake;
import frc.team5115.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class Bindings {
    private final CommandXboxController driveJoy;
    private final CommandXboxController manipJoy;

    private boolean robotRelative = false;
    private boolean slowMode = false;

    public Bindings() {
        driveJoy = new CommandXboxController(0);
        // If in single mode, both Controller objects reference the controller on port 0
        manipJoy = Constants.SINGLE_MODE ? driveJoy : new CommandXboxController(1);
    }

    public boolean joysticksConnected() {
        return driveJoy.isConnected() && manipJoy.isConnected();
    }

    private Command offsetGyro(Drivetrain drivetrain) {
        return Commands.runOnce(() -> drivetrain.offsetGyro(), drivetrain).ignoringDisable(true);
    }

    /**
     * If either controller presses down on the d-pad, disable automation.
     *
     * @return a Trigger that rises when automation enables and falls when automation is disabled.
     */
    private Trigger automationEnabled() {
        return (manipJoy.pov(180))
                .or(manipJoy.pov(135))
                .or(manipJoy.pov(225))
                .or(driveJoy.pov(180))
                .or(driveJoy.pov(135))
                .or(driveJoy.pov(225))
                .negate();
    }

    public void configureButtonBindings(
            Drivetrain drivetrain,
            Intake intake,
            Agitator agitator,
            Indexer indexer,
            Shooter shooter,
            DoubleSupplier shooterSpeed,
            Trigger safeToShoot) {
        drivetrain.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drivetrain,
                        () -> robotRelative,
                        () -> slowMode,
                        () -> -driveJoy.getLeftY(),
                        () -> -driveJoy.getLeftX(),
                        () -> -driveJoy.getRightX()));

        driveJoy.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driveJoy.leftBumper().onTrue(setRobotRelative(true)).onFalse(setRobotRelative(false));
        driveJoy.rightBumper().onTrue(setSlowMode(true)).onFalse(setSlowMode(false));
        driveJoy.start().onTrue(offsetGyro(drivetrain));

        manipJoy.back().whileTrue(DriveCommands.vomit(agitator, indexer, intake));

        intake.setDefaultCommand(intake.intake());
        agitator.setDefaultCommand(agitator.slow());
        indexer.setDefaultCommand(indexer.reject());

        // Right trigger smart shoots
        manipJoy
                .rightTrigger()
                .whileTrue(
                        DriveCommands.smartShoot(
                                drivetrain, agitator, indexer, shooter, Shooter.Requester.ManualShoot));

        // Left trigger dumb shoots
        manipJoy.leftTrigger().whileTrue(DriveCommands.dumbShoot(agitator, indexer, shooter));

        // While in alliance zone request to spin up shooter
        drivetrain
                .inAllianceZone()
                .and(automationEnabled())
                .whileTrue(shooter.requestSpinUp(Shooter.Requester.InAllianceZone));

        // While holding a, spin up shooter
        driveJoy.a().whileTrue(shooter.requestSpinUp(Shooter.Requester.ManualSpinUp));

        // While in the subzone, or holding a, lock on
        drivetrain
                .inSubZone()
                .and(automationEnabled())
                .or(driveJoy.a())
                .whileTrue(
                        DriveCommands.lockedOnHub(
                                shooter,
                                drivetrain,
                                () -> slowMode,
                                () -> -driveJoy.getLeftY(),
                                () -> -driveJoy.getLeftX()));

        safeToShoot
                .onTrue(rumble(Constants.RUMBLE_STRENGTH))
                .onFalse(rumble(0))
                .and(automationEnabled())
                .whileTrue(
                        DriveCommands.smartShoot(
                                drivetrain, agitator, indexer, shooter, Shooter.Requester.SafeShoot));
    }

    public void configureBlingBindings(
            Bling bling,
            Drivetrain drivetrain,
            Indexer indexer,
            Shooter shooter,
            RobotFaults faults,
            Trigger safeToShoot) {
        bling.setDefaultCommand(bling.allianceKITT());
        drivetrain.inAllianceZone().whileTrue(bling.allianceScrollIn());
        drivetrain.inSubZone().whileTrue(bling.purpleScrollIn());
        safeToShoot.whileTrue(bling.purpleFlashing());
        new Trigger(indexer::isIndexing).whileTrue(bling.whiteScrollIn());
        new Trigger(faults::hasFaults).whileTrue(bling.faultFlash().ignoringDisable(true));
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
                    driveJoy.setRumble(GenericHID.RumbleType.kBothRumble, value);
                    if (manipJoy != null) {
                        manipJoy.setRumble(GenericHID.RumbleType.kBothRumble, value);
                    }
                });
    }
}
