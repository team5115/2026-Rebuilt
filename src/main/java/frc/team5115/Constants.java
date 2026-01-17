package frc.team5115;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
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

    public static final Pose2d SIM_INIT_POSE = new Pose2d(3, 3, Rotation2d.kZero);

    public static final boolean SINGLE_MODE = true;
    public static final double RUMBLE_STRENGTH = 0.5;

    public static final byte INTAKE_MOTOR_ID = 12;
    public static final byte SHOOTER_MOTOR_ID = -1; // TODO determine motor ID
    public static final byte INDEXER_MOTOR_ID = -1; // TODO determine motor ID

    public static final byte LED_STRIP_PWM_ID = 0;

    public static final double LOOP_PERIOD_SECS = 0.02;

    public static final double INDEX_SPEED = 0.22;

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

        // TODO set TRACK_WIDTH to its new value for this year
        private static final double TRACK_WIDTH = Units.inchesToMeters(26.25);
        public static final double TRACK_WIDTH_X = TRACK_WIDTH;
        public static final double TRACK_WIDTH_Y = TRACK_WIDTH;
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
        private static final List<Pose2d> leftScoringPoses = new ArrayList<Pose2d>();
        private static final List<Pose2d> rightScoringPoses = new ArrayList<Pose2d>();
        private static final List<Pose2d> centerScoringPoses = new ArrayList<Pose2d>();
        private static final List<Pose2d> sourcePoses = new ArrayList<Pose2d>();

        public enum Side {
            LEFT(leftScoringPoses),
            RIGHT(rightScoringPoses),
            CENTER(centerScoringPoses);

            public final List<Pose2d> poses;

            Side(final List<Pose2d> poses) {
                this.poses = poses;
            }
        }

        private static final double forwardOffset = 0.46; // distance from the april tag
        private static final Transform2d transformLeft =
                new Transform2d(new Translation2d(forwardOffset, -0.36), Rotation2d.k180deg);
        private static final Transform2d transformRight =
                new Transform2d(new Translation2d(forwardOffset, +0.0), Rotation2d.k180deg);
        private static final Transform2d transformCenter =
                new Transform2d(new Translation2d(forwardOffset, -0.25), Rotation2d.k180deg);

        public static Pose2d getNearestScoringSpot(final Pose2d robot, final Side side) {
            return robot.nearest(side.poses);
        }

        public static Pose2d getNearestSource(final Pose2d robot) {
            return robot.nearest(sourcePoses);
        }

        private static void precomputeSourcePoses() {
            final double blueX = 1.427;
            final double redX = 16.108;
            final double lowY = 0.787;
            final double highY = 7.200;

            sourcePoses.add(new Pose2d(blueX, lowY, Rotation2d.fromDegrees(55)));
            sourcePoses.add(new Pose2d(blueX, highY, Rotation2d.fromDegrees(-55)));
            sourcePoses.add(new Pose2d(redX, lowY, Rotation2d.fromDegrees(180 - 55)));
            sourcePoses.add(new Pose2d(redX, highY, Rotation2d.fromDegrees(55 - 180)));
        }

        public static void precomputeAlignmentPoses() {
            for (final AprilTag tag : VisionConstants.FIELD_LAYOUT.getTags()) {
                if (((tag.ID >= 6 && tag.ID <= 11) || (tag.ID >= 17 && tag.ID <= 22))) {
                    final Pose2d tagPose = tag.pose.toPose2d();
                    leftScoringPoses.add(tagPose.transformBy(transformLeft));
                    rightScoringPoses.add(tagPose.transformBy(transformRight));
                    centerScoringPoses.add(tagPose.transformBy(transformCenter));
                }
            }
            precomputeSourcePoses();
        }

        // Calculated using AndyMark april tag json combined with field cad
        // 0.818973 is the distance from the center of the reef to the center of an edge
        private static final Translation2d blueReefCenter =
                new Translation2d(5.321046 - 0.818973, 4.02082);
        private static final Translation2d redReefCenter =
                new Translation2d(12.227306 + 0.818973, 4.02082);

        public static double getReefX(boolean isRedAlliance) {
            return isRedAlliance ? redReefCenter.getX() : blueReefCenter.getX();
        }

        public static double getReefY(boolean isRedAlliance) {
            return isRedAlliance ? redReefCenter.getY() : blueReefCenter.getY();
        }
    }

    public static class VisionConstants {

        // private static AprilTagFieldLayout loadReefOnlyFieldLayout() {
        //     try {
        //         // throw new IOException();
        //         return new AprilTagFieldLayout(
        //                 Filesystem.getDeployDirectory().getAbsolutePath()
        //                         + File.separatorChar
        //                         + "reef_only.json");
        //     } catch (IOException e) {
        //         e.printStackTrace();
        //         return loadFullField();
        //     }
        // }

        private static AprilTagFieldLayout loadFullField() {
            return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        }

        public static final AprilTagFieldLayout FIELD_LAYOUT = loadFullField();

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
        public static final double distanceThreshold = 1.5; // meters
        public static final double angleThreshold = 10.0; // degrees
        public static final double tagYawThreshold = 4.0; // degrees
        public static final double zTranslationThreshold = 0.15; // meters
        public static final double ambiguityThreshold = 0.5;
        // every tag beyond seeing two tags gives us an extra meter of trusted distance
        public static final double multiTagDistanceFactor = 1.0;
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
