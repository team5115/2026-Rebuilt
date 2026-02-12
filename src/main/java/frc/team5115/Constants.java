package frc.team5115;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

public final class Constants {
    private static final boolean isReplay = false;
    public static final Mode currentMode =
            RobotBase.isReal() ? Mode.REAL : (isReplay ? Mode.REPLAY : Mode.SIM);

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final Pose2d SIM_INIT_POSE = new Pose2d(1.55, 0.8, Rotation2d.kZero);

    public static final boolean SINGLE_MODE = true;
    public static final double RUMBLE_STRENGTH = 0.5;

    // TODO set motor IDs on real robot
    public static final byte INTAKE_MOTOR_ID = 11;
    public static final byte AGITATOR_MOTOR_ID = 12;
    public static final byte INDEXER_MOTOR_ID = 13;
    public static final byte SHOOTER_MOTOR_ID = 14;

    public static final byte LED_STRIP_PWM_ID = 0;

    public static final double LOOP_PERIOD_SECS = 0.02;

    // TODO determine digestion speeds
    public static final double INTAKE_SPEED = 0.5;
    public static final double INTAKE_VOMIT_SPEED = -1.0;

    public static final double AGITATOR_FAST_SPEED = 0.4;
    public static final double AGITATOR_SLOW_SPEED = 0.1;
    public static final double AGITATOR_PAUSE_SEC = 0.3;
    public static final double AGITATOR_VOMIT_SPEED = -0.5;

    public static final double INDEX_SPEED = 0.4;
    public static final double INDEX_REJECT_SPEED = -0.25;
    public static final double INDEX_VOMIT_SPEED = -0.25;

    public static final Distance ALLIANCE_TRANSLATION = Meters.of(11.915775);

    public static final Ellipse2d BLUE_SUB_ZONE =
            new Ellipse2d(new Translation2d(Meters.of(4.625), Meters.of(4.035)), Meters.of(3.0));

    public static final Rectangle2d BLUE_ALLIANCE_ZONE =
            new Rectangle2d(new Translation2d(), new Translation2d(Meters.of(3.0), Meters.of(8.09625)));

    public static final Ellipse2d RED_SUB_ZONE =
            new Ellipse2d(
                    new Translation2d(Meters.of(4.625).plus(ALLIANCE_TRANSLATION), Meters.of(4.035)),
                    Meters.of(3.0));

    public static final Rectangle2d RED_ALLIANCE_ZONE =
            new Rectangle2d(
                    new Translation2d(),
                    new Translation2d(Meters.of(4.0).plus(ALLIANCE_TRANSLATION), Meters.of(8.09625)));

    public static class SwerveConstants {
        public static final byte FRONT_LEFT_DRIVE_ID = 6;
        public static final byte FRONT_RIGHT_DRIVE_ID = 4;
        public static final byte BACK_LEFT_DRIVE_ID = 10;
        public static final byte BACK_RIGHT_DRIVE_ID = 8;

        public static final byte FRONT_LEFT_TURN_ID = 5;
        public static final byte FRONT_RIGHT_TURN_ID = 3;
        public static final byte BACK_LEFT_TURN_ID = 9;
        public static final byte BACK_RIGHT_TURN_ID = 7;

        private static RobotConfig ROBOT_CONFIG = null;

        public static RobotConfig getRobotConfig() {
            if (ROBOT_CONFIG == null) {
                try {
                    ROBOT_CONFIG = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            return ROBOT_CONFIG;
        }

        public static final double MAX_LINEAR_SPEED = 5; // meters per second

        // 29" wide, 25.75" front to back
        // ! Reminder that X is front-back, Y is left-right
        private static final Distance FRAME_WIDTH_X = Inches.of(25.75);
        private static final Distance FRAME_WIDTH_Y = Inches.of(29);
        private static final Distance BUMPER_DEPTH = Inches.of(2.75);
        private static final Distance MODULE_INSET = Inches.of(1.75);

        public static final Distance BUMPER_WIDTH_X = FRAME_WIDTH_X.plus(BUMPER_DEPTH.times(2));
        public static final Distance BUMPER_WIDTH_Y = FRAME_WIDTH_Y.plus(BUMPER_DEPTH.times(2));

        public static final double TRACK_WIDTH_X =
                FRAME_WIDTH_X.minus(MODULE_INSET.times(2)).in(Meters);
        public static final double TRACK_WIDTH_Y =
                FRAME_WIDTH_Y.minus(MODULE_INSET.times(2)).in(Meters);

        public static final double DRIVE_BASE_RADIUS =
                Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
        public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);

        // Required for inverse kinematics. +x is forward, +y is left
        // The module order, as with everywhere else, is FL, FR, BL, BR
        public static final Translation2d[] MODULE_TRANSLATIONS =
                new Translation2d[] {
                    new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                    new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / -2.0),
                    new Translation2d(TRACK_WIDTH_X / -2.0, TRACK_WIDTH_Y / 2.0),
                    new Translation2d(TRACK_WIDTH_X / -2.0, TRACK_WIDTH_Y / -2.0)
                };

        public static final Rotation2d FRONT_LEFT_ANGULAR_OFFSET = Rotation2d.fromDegrees(90);
        public static final Rotation2d FRONT_RIGHT_ANGULAR_OFFSET = Rotation2d.fromDegrees(180);
        public static final Rotation2d BACK_LEFT_ANGULAR_OFFSET = Rotation2d.fromDegrees(0);
        public static final Rotation2d BACK_RIGHT_ANGULAR_OFFSET = Rotation2d.fromDegrees(270);

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear
        // 15 teeth on the bevel pinion, 13 teeth on the driving motor
        public static final double DrivingMotorReduction = (45.0 * 22.0) / (13.0 * 15.0);

        public static final int DrivingMotorAutoCurrentLimit = 60; // amp
        public static final int DrivingMotorTeleopCurrentLimit = 50; // amps, lower than in auto
        public static final int TurningMotorCurrentLimit = 20; // amps
    }

    public static class AutoConstants {
        public static final double MAX_AUTOALIGN_LINEAR_SPEED = 4.0; // m/s
        public static final Mass ROBOT_MASS = Pounds.of(83); // TODO update weight estimation

        private static final Translation2d BLUE_HUB = new Translation2d(4.63, 4.03);
        private static final Translation2d RED_HUB = new Translation2d(11.92, 4.03);

        /**
         * Get the distance from the robot to our alliance hub
         *
         * @param robot the position of the robot on the field
         * @return the distance from the robot to the hub
         */
        public static double distanceToHub(Pose2d robot) {
            return getHubPosition().getDistance(robot.getTranslation());
        }

        public static Translation2d getHubPosition() {
            return isRedAlliance() ? RED_HUB : BLUE_HUB;
        }

        public static boolean isInZone(Pose2d robot, Rectangle2d zone) {
            return zone.contains(robot.getTranslation());
        }

        public static boolean isInAllianceZone(Pose2d robot) {
            return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                            ? RED_ALLIANCE_ZONE
                            : BLUE_ALLIANCE_ZONE)
                    .contains(robot.getTranslation());
        }

        public static boolean isInSubZone(Pose2d robot) {
            return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                    ? RED_ALLIANCE_ZONE.contains(robot.getTranslation())
                            && RED_SUB_ZONE.contains(robot.getTranslation())
                    : BLUE_ALLIANCE_ZONE.contains(robot.getTranslation())
                            && BLUE_SUB_ZONE.contains(robot.getTranslation()));
        }
    }

    public static class VisionConstants {

        private static AprilTagFieldLayout loadCustomFieldLayout() {
            try {
                return new AprilTagFieldLayout(
                        Filesystem.getDeployDirectory().getAbsolutePath()
                                + "/apriltag-fields/custom-2026-rebuilt-andymark.json");
            } catch (java.io.IOException e) {
                e.printStackTrace();
                return loadFullField();
            }
        }

        private static AprilTagFieldLayout loadFullField() {
            return AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        }

        public static final AprilTagFieldLayout FIELD_LAYOUT = loadCustomFieldLayout();

        // Camera sim values
        public static final int WIDTH_PX = 1280;
        public static final int HEIGHT_PX = 720;
        public static final double DIAG_FOV_DEGREES = 90;
        public static final double AVG_ERR_PX = 0.5;
        public static final double STD_DEV_ERR_PX = 0;
        public static final double FPS = 30;
        public static final double AVG_LATENCY_MS = 30;
        public static final double STD_DEV_LATENCY_MS = 10;

        // Pose filtering values
        public static final double distanceThreshold = 2.0; // meters
        public static final double angleThreshold = 50.0; // degrees
        public static final double tagYawThreshold = 4.0; // degrees
        public static final double zTranslationThreshold = 0.15; // meters
        public static final double ambiguityThreshold = 0.5;
        // every tag beyond seeing two tags gives us an extra 30% of our threshold
        public static final double multiTagDistanceFactor = 0.3;

        /*
        The coordinate system for the camera to robot transforms is somewhat confusing.
        All lengths are in meters, and angles are in degrees.
        Positions are relative to the center of the robot.
        Positive X means that the camera is towards the front of the robot.
        Positive Y is directed to the left of the robot.
        Positive yaw points to the left, i.e. 90 degrees in yaw is directly pointed left.
        Positive pitch is actually pointed down, which is VERY important to remember.
        Positive roll is the camera rolling towards the robot's right side.
        */
        // TODO camera transform is subject to change
        private static final double camX = Units.inchesToMeters(+12.043);
        private static final double camZ = Units.inchesToMeters(+20.181);

        private static final double leftY = Units.inchesToMeters(+11.719);
        private static final double rightY = Units.inchesToMeters(-11.719);

        private static final Angle roll = Degrees.of(+0);
        private static final Angle pitch = Degrees.of(-22);

        private static final Angle leftYaw = Degrees.of(-8.297);
        private static final Angle rightYaw = Degrees.of(+8.297);

        public static final Transform3d LEFT_CAM_TO_ROBOT =
                new Transform3d(camX, leftY, camZ, new Rotation3d(roll, pitch, leftYaw));
        public static final Transform3d RIGHT_CAM_TO_ROBOT =
                new Transform3d(camX, rightY, camZ, new Rotation3d(roll, pitch, rightYaw));
    }

    public static boolean isHubEnabled() {
        final double matchTime = Timer.getMatchTime();
        final String gameData = DriverStation.getGameSpecificMessage();
        final var alliance = DriverStation.getAlliance();
        return isHubEnabledTest(matchTime, gameData, alliance);
    }

    @AutoLogOutput(key = "IsRedAlliance")
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red;
    }

    private static boolean isHubEnabledTest(
            double matchTime, String gameData, Optional<Alliance> alliance) {
        if (alliance.isEmpty()) {
            return true;
        }

        final boolean isBlue = alliance.get().equals(Alliance.Blue);
        if (matchTime <= 30) {
            return true;
            // auto and endgame
        }
        if (matchTime >= 130) {
            return true;
        }

        boolean blueWonAuto;
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    blueWonAuto = true;
                    break;
                case 'R':
                    blueWonAuto = false;
                    break;
                default:
                    System.err.println("Bad game data");
                    return true;
            }
        } else {
            return true;
        }

        if (matchTime >= 105) {
            return blueWonAuto != isBlue;
        }
        if (matchTime >= 80) {
            return blueWonAuto == isBlue;
        }
        if (matchTime >= 55) {
            return blueWonAuto != isBlue;
        }
        if (matchTime > 30) {
            return blueWonAuto == isBlue;
        }
        System.err.println("wtf is happening");
        return true;
    }
}
