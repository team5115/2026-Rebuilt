package frc.team5115;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkMax;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.indexer.Indexer;
import frc.team5115.subsystems.intake.Intake;
import frc.team5115.subsystems.shooter.Shooter;
import frc.team5115.subsystems.vision.PhotonVision;
import java.util.ArrayList;

public class RobotFaults {
    private static final String NO_FAULTS = "No Faults";
<<<<<<< HEAD
    // public final String sparkFaults;
    // public final boolean cameraDisconnected;
    // public final boolean joysticksDisconnected;
    // public final boolean gyroDisconnected;
    // public final boolean drivetrainNull;
    // public final boolean visionNull;
    // public final boolean climberNull;
    // public final boolean elevatorNull;
    // public final boolean dispenserNull;
    // public final boolean intakeNull;
    // public final boolean elevatorShorted;

    public final ArrayList<String> sparkFaults = new ArrayList<String>();
    // private final String cachedToString;

    public RobotFaults(
            String sparkFaults
            ) {
        this.sparkFaults = sparkFaults;
    }

    // public String cacheString() {
    //     final StringBuilder builder = new StringBuilder();
    //     if (!sparkFaults.isEmpty()) {
    //         builder.append("SparkFaults:[ ");
    //         builder.append(sparkFaults);
    //         builder.append("] ; ");
    //     }
    //     if (cameraDisconnected) {
    //         builder.append("CameraDisconnected; ");
    //     }
    //     if (joysticksDisconnected) {
    //         builder.append("JoysticksDisconnected; ");
    //     }
    //     if (gyroDisconnected) {
    //         builder.append("GyroDisconnected; ");
    //     }
    //     if (drivetrainNull) {
    //         builder.append("DrivetrainNull; ");
    //     }
    //     if (visionNull) {
    //         builder.append("VisionNull; ");
    //     }
    //     if (climberNull) {
    //         builder.append("ClimberNull; ");
    //     }
    //     if (elevatorNull) {
    //         builder.append("ElevatorNull; ");
    //     }
    //     if (dispenserNull) {
    //         builder.append("DispenserNull; ");
    //     }
    //     if (intakeNull) {
    //         builder.append("IntakeNull; ");
    //     }
    //     if (elevatorShorted) {
    //         builder.append("5V Short; ");
    //     }
    //     if (builder.isEmpty()) {
    //         return NO_FAULTS;
    //     } else {
    //         return "HAS FAULTS! " + builder.toString();
    //     }
    // }
=======
    public final String sparkFaults;
    public final boolean cameraDisconnected;
    public final boolean joysticksDisconnected;
    public final boolean gyroDisconnected;
    public final boolean drivetrainNull;
    public final boolean visionNull;
    public final boolean intakeNull;
    public final boolean shooterNull;
    public final boolean indexerNull;
    private final String cachedToString;

    public RobotFaults(
            String sparkFaults,
            boolean cameraDisconnected,
            boolean joysticksDisconnected,
            boolean gyroDisconnected,
            boolean drivetrainNull,
            boolean visionNull,
            boolean intakeNull,
            boolean shooterNull,
            boolean indexerNull) {
        this.sparkFaults = sparkFaults;
        this.cameraDisconnected = cameraDisconnected;
        this.joysticksDisconnected = joysticksDisconnected;
        this.gyroDisconnected = gyroDisconnected;
        this.drivetrainNull = drivetrainNull;
        this.visionNull = visionNull;
        this.intakeNull = intakeNull;
        this.shooterNull = shooterNull;
        this.indexerNull = indexerNull;
        cachedToString = cacheString();
    }

    public String cacheString() {
        final StringBuilder builder = new StringBuilder();
        if (!sparkFaults.isEmpty()) {
            builder.append("SparkFaults:[ ");
            builder.append(sparkFaults);
            builder.append("] ; ");
        }
        if (cameraDisconnected) {
            builder.append("CameraDisconnected; ");
        }
        if (joysticksDisconnected) {
            builder.append("JoysticksDisconnected; ");
        }
        if (gyroDisconnected) {
            builder.append("GyroDisconnected; ");
        }
        if (drivetrainNull) {
            builder.append("DrivetrainNull; ");
        }
        if (visionNull) {
            builder.append("VisionNull; ");
        }
        if (intakeNull) {
            builder.append("IntakeNull; ");
        }
        if (builder.isEmpty()) {
            return NO_FAULTS;
        } else {
            return "HAS FAULTS! " + builder.toString();
        }
    }
>>>>>>> 14d44bac52f24fa94c5e4f23c897820728a6a78b

    @Override
    public String toString() {
        return sparkFaults.toString();
    }

    public boolean hasFaults() {
        return sparkFaults.size() > 0;
    }

    public static void fromSubsystems(
            Drivetrain drivetrain,
            PhotonVision vision,
            Intake intake,
            Shooter shooter,
            Indexer indexer,
            boolean joysticksConnected) {

        ArrayList<SparkMax> sparks = new ArrayList<>();
        if (drivetrain != null) {
            drivetrain.getSparks(sparks);
        }
        if (intake != null) {
            intake.getSparks(sparks);
        }
        if (shooter != null) {
            shooter.getSparks(sparks);
        }
        if (indexer != null) {
            indexer.getSparks(sparks);
        }
        sparkFaults = new ArrayList<String>();
        for (var spark : sparks) {
<<<<<<< HEAD
            appendSparkFaults(spark.getFaults(), spark.getDeviceId());
        };
=======
            appendSparkFaults(sparkFaults, spark.getFaults(), spark.getDeviceId());
        }
        return new RobotFaults(
                sparkFaults.toString(),
                vision == null ? true : vision.areAnyCamerasDisconnected(),
                !joysticksConnected,
                drivetrain == null ? true : !drivetrain.isGyroConnected(),
                drivetrain == null,
                vision == null,
                intake == null,
                shooter == null,
                indexer == null);
>>>>>>> 14d44bac52f24fa94c5e4f23c897820728a6a78b
    }

    private static void appendSparkFaults(Faults faults, int id) {

        final StringBuilder builder = new StringBuilder();
        if (faults.other) {
            builder.append("OtherFault,");
        }
        if (faults.motorType) {
            builder.append("MotorTypeFault,");
        }
        if (faults.sensor) {
            builder.append("SensorFault,");
        }
        if (faults.can) {
            builder.append("CanFault,");
        }
        if (faults.temperature) {
            builder.append("TemperatureFault,");
        }
        if (faults.gateDriver) {
            builder.append("GateDriverFault,");
        }
        if (faults.escEeprom) {
            builder.append("EscEepromFault,");
        }
        if (faults.firmware) {
            builder.append("FirmwareFault,");
        }
        if (!builder.isEmpty()) {
            // SparkFaults:[ { ID_0,GateDriverFault },{ ID_1,FirmwareFault}, ] ;
            sparkFaults.add("{ ID_");
            sparkFaults.add(id);
            sparkFaults.add(",");
            sparkFaults.add(builder);
            sparkFaults.add(" },");
        }
    }
}
