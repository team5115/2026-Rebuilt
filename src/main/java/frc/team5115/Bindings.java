package frc.team5115;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5115.commands.DriveCommands;
import frc.team5115.subsystems.drive.Drivetrain;

public class Bindings {
    private final CommandXboxController driveJoy;
    private final CommandXboxController manipJoy;

    private final Drivetrain drivetrain;

    // private final Intake intake;
    // private final Agitator agitator;
    // private final Indexer indexer;
    // private final Shooter shooter;

    public Bindings(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // this.intake = intake;
        // this.agitator = agitator;
        // this.indexer = indexer;
        // this.shooter = shooter;

        // If in single mode, both Controller objects reference the controller on port 0
        driveJoy = new CommandXboxController(0);
        manipJoy = Constants.SINGLE_MODE ? driveJoy : new CommandXboxController(1);
    }

    public boolean joysticksConnected() {
        return driveJoy.isConnected() && manipJoy.isConnected();
    }

    private Command offsetGyro() {
        return Commands.runOnce(() -> drivetrain.zeroGyro(), drivetrain).ignoringDisable(true);
    }

    /**
     * Automation is enabled UNLESS one of the following conditions is true:
     *
     * <ol>
     *   <li>either driver presses on the d-pad
     *   <li>auto is enabled
     *   <li>the robot is disabled
     *   <li>{@code Constants.DISABLE_AUTOMATION} is true
     * </ol>
     *
     * @return a Trigger that rises when automation enables and falls when automation is disabled.
     */
    public Trigger automationEnabled() {
        return (driveJoy.povCenter().negate())
                .or(manipJoy.povCenter().negate())
                .or(DriverStation::isAutonomousEnabled)
                .or(DriverStation::isDisabled)
                .or(() -> Constants.DISABLE_AUTOMATION)
                .negate();
    }

    // /**
    //  * AutoHubLock is enabled if all of the following conditions are met:
    //  *
    //  * <ol>
    //  *   <li>the robot is inside the sub-zone
    //  *   <li>automation is enabled (see: {@link #automationEnabled()})
    //  *   <li>neither driver presses the left trigger
    //  * </ol>
    //  *
    //  * @return a Trigger that indicates if automatic hub lock is enabled
    //  */
    // private Trigger autoHubLockEnabled() {
    //     return drivetrain
    //             .inSubZone()
    //             .and(automationEnabled())
    //             .and(manipJoy.leftTrigger().negate())
    //             .and(driveJoy.leftTrigger().negate());
    // }

    /**
     * Determines if it safe to shoot. Checks that the following conditions are true:
     *
     * <ol>
     *   <li>is the robot in the sub-zone?
     *   <li>is the hub active?
     *   <li>~~is the shooter spun up to speed?~~
     *   <li>is the drivetrain heading locked onto the hub?
     *   <li>are the drivetrain linear and rotational speeds close enough to zero?
     * </ol>
     *
     * @param drivetrain
     * @param shooter
     * @return a Trigger of if it's safe to shoot
     */
    public Trigger safeToShoot() {
        return drivetrain
                .inSubZone()
                .and(Constants::isHubActive)
                // .and(shooter::atSetpoint)
                .and(drivetrain::lockedOnHub)
                .and(() -> drivetrain.movingWithinTolerance(0.2, 0.5));
    }

    // /**
    //  * Is the drivetrain moving slow enough to consider spinning up the shooter? This trigger is
    //  * primarily used in instances of checking whether the shooter should spin to a LOW speed.
    //  *
    //  * @return a debounced Trigger
    //  */
    // private Trigger slowEnoughToSpinUp() {
    //     return new Trigger(() -> drivetrain.movingWithinTolerance(1.0, 2.0)).debounce(0.2);
    // }

    public void configureButtonBindings() {
        final Trigger slowMode = driveJoy.rightBumper();
        drivetrain.setDefaultCommand(
                DriveCommands.fieldRelativeHeadingDrive(
                        drivetrain,
                        slowMode,
                        () -> -driveJoy.getLeftY(),
                        () -> -driveJoy.getLeftX(),
                        () -> -driveJoy.getRightY(),
                        () -> -driveJoy.getRightX()));

        // Hold left bumper to drive robot relative
        driveJoy
                .leftBumper()
                .whileTrue(
                        DriveCommands.joystickDrive(
                                drivetrain,
                                () -> true,
                                slowMode,
                                () -> -driveJoy.getLeftY(),
                                () -> -driveJoy.getLeftX(),
                                () -> -driveJoy.getRightX()));

        driveJoy.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driveJoy.start().onTrue(offsetGyro());

        // manipJoy.back().whileTrue(DriveCommands.vomit(agitator, indexer, intake));

        // intake.setDefaultCommand(intake.intake());
        // agitator.setDefaultCommand(agitator.slow());
        // indexer.setDefaultCommand(indexer.reject());

        // Right trigger smart shoots
        // manipJoy
        //         .rightTrigger()
        //         .whileTrue(
        //                 DriveCommands.smartShoot(
        //                         drivetrain, agitator, indexer, shooter, SpeedRequest.ManualShoot));

        // // B shoots blind
        // manipJoy.b().whileTrue(DriveCommands.blindShoot(agitator, indexer, shooter, shooterSpeed));

        // While in alliance zone request to spin up shooter
        // drivetrain
        //         .inAllianceZone()
        // .and(automationEnabled())
        // .and(slowEnoughToSpinUp())
        // .whileTrue(shooter.requestSpinUp(SpeedRequest.InAllianceZone));

        // While autoHubLock is enabled, or holding a, lock on
        // autoHubLockEnabled()
        //         .or(driveJoy.a())
        //         .whileTrue(
        //                 DriveCommands.lockedOnHub(
        //                         shooter,
        //                         drivetrain,
        //                         () -> slowMode,
        //                         () -> -driveJoy.getLeftY(),
        //                         () -> -driveJoy.getLeftX()));

        // If driver is locking onto hub spin up shooter
        // driveJoy
        //         .a()
        //         // .and(slowEnoughToSpinUp())
        //         .whileTrue(shooter.requestSpinUp(SpeedRequest.ManualSpinUp));

        // Rumble whenever safe to shoot
        // If automation enabled, then shoot automatically
        // safeToShoot()
        //         .onTrue(rumble(Constants.RUMBLE_STRENGTH))
        //         .onFalse(rumble(0))
        //         .and(automationEnabled())
        //         .whileTrue(
        //                 DriveCommands.smartShoot(
        //                         drivetrain, agitator, indexer, shooter, SpeedRequest.SafeShoot));

        // driveJoy.x().whileTrue(shooter.moveActuators(linearPosition));
    }

    // public void configureBlingBindings(Bling bling, RobotFaults faults) {
    //     bling.setDefaultCommand(bling.allianceKITT());
    //     drivetrain.inAllianceZone().whileTrue(bling.allianceScrollIn());
    //     drivetrain
    //             .inAllianceZone()
    //             .negate()
    //             .and(Constants::isHubActive)
    //             .whileTrue(bling.allianceWhiteFlashing());
    //     drivetrain.inSubZone().whileTrue(bling.purpleScrollIn());
    //     safeToShoot().whileTrue(bling.purpleFlashing());
    //     // new Trigger(indexer::isIndexing).whileTrue(bling.whiteScrollIn());
    //     new Trigger(faults::hasFaults).whileTrue(bling.faultFlash().ignoringDisable(true));
    // }

    // private Command rumble(double value) {
    //     return Commands.runOnce(
    //             () -> {
    //                 driveJoy.setRumble(GenericHID.RumbleType.kBothRumble, value);
    //                 if (manipJoy != null) {
    //                     manipJoy.setRumble(GenericHID.RumbleType.kBothRumble, value);
    //                 }
    //             });
    // }
}
