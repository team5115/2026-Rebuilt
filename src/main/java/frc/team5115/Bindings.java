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
    private final CommandXboxController joyDrive;
    private final CommandXboxController joyManip;

    private boolean robotRelative = false;
    private boolean slowMode = false;

    public Bindings() {
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

    public void configureButtonBindings(
            Drivetrain drivetrain,
            Intake intake,
            Agitator agitator,
            Indexer indexer,
            Shooter shooter,
            DoubleSupplier shooterSpeed) {
        // drive control
        drivetrain.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drivetrain,
                        () -> robotRelative,
                        () -> slowMode,
                        () -> -joyDrive.getLeftY(),
                        () -> -joyDrive.getLeftX(),
                        () -> -joyDrive.getRightX()));

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
        joyDrive.a().whileTrue(shooter.supplySetpoint(shooterSpeed)).onFalse(shooter.setSetpoint(0));

        // Slowly agitate and reject by default
        agitator.setDefaultCommand(agitator.slow());
        indexer.setDefaultCommand(indexer.reject());

        // TODO do we want to always intake?
        intake.setDefaultCommand(intake.intake().repeatedly());

        if (Constants.SINGLE_MODE) {
            configureSingleMode(drivetrain, intake, agitator, indexer, shooter);
        } else {
            configureDualMode(drivetrain, intake, agitator, indexer, shooter);
        }
    }

    private void configureSingleMode(
            Drivetrain drivetrain, Intake intake, Agitator agitator, Indexer indexer, Shooter shooter) {
        joyDrive
                .a()
                .whileTrue(
                        DriveCommands.lockedOnHub(
                                shooter,
                                drivetrain,
                                () -> slowMode,
                                () -> -joyDrive.getLeftY(),
                                () -> -joyDrive.getLeftX()));

        joyDrive
                .rightTrigger()
                .whileTrue(DriveCommands.smartShoot(drivetrain, agitator, indexer, shooter));

        joyDrive.leftTrigger().whileTrue(DriveCommands.dumbShoot(agitator, indexer, shooter));

        joyDrive.back().whileTrue(DriveCommands.vomit(agitator, indexer, intake));

        new Trigger(
                        () -> {
                            return true;
                        })
                .whileTrue(shooter.maintainSpeed(drivetrain::getDistanceToHub));
    }

    public void configureBlingBindings(Bling bling, Drivetrain drivetrain, RobotFaults faults) {
        bling.setDefaultCommand(bling.redKITT().ignoringDisable(true));

        // TODO update bling bindings for Rebuilt game
        drivetrain.aligningToGoal().whileTrue(bling.yellowScrollIn());
        drivetrain.alignedAtGoalTrigger().whileTrue(bling.whiteScrollIn());

        new Trigger(faults::hasFaults).whileTrue(bling.faultFlash().ignoringDisable(true));
    }

    private void configureDualMode(
            Drivetrain drivetrain, Intake intake, Agitator agitator, Indexer indexer, Shooter shooter) {
        joyManip.back().whileTrue(DriveCommands.vomit(agitator, indexer, intake));
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
