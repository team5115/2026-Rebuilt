package frc.team5115;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.vision.PhotonVision;
import frc.team5115.util.MotorContainer;
import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class RobotFaults {
    // private static final String NO_FAULTS = "No Faults";
    public final Drivetrain drivetrain;
    public final PhotonVision vision;
    public final MotorContainer[] motorContainers;
    public final BooleanSupplier joysticksConnected;

    private final Timer faultUpdateTimer = new Timer();

    public final ArrayList<SparkMax> sparks = new ArrayList<SparkMax>();
    public final ArrayList<String> faultArray = new ArrayList<String>();

    public RobotFaults(
            Drivetrain drivetrain,
            PhotonVision vision,
            BooleanSupplier joysticksConnected,
            MotorContainer... motorContainers) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.joysticksConnected = joysticksConnected;
        this.motorContainers = motorContainers;
        this.faultUpdateTimer.start();

        if (drivetrain != null) {
            this.sparks.addAll(drivetrain.getSparks());
        }
        for (MotorContainer container : motorContainers) {
            if (container != null) {
                sparks.addAll(container.getSparks());
            }
        }
    }

    @Override
    public String toString() {
        return "SparkFaults:[ "
                + formatSparkFaults(getSparkFaults())
                + "] ; "
                + getSubsystemFaults().toString();
    }

    public boolean hasFaults() {
        return getSparkFaults().size() > 0 || getSubsystemFaults().size() > 0;
    }

    public ArrayList<String> getSubsystemFaults() {
        ArrayList<String> faultArray = new ArrayList<>();
        if (vision == null || vision.areAnyCamerasDisconnected()) {
            faultArray.add("CameraDisconnected");
        }
        if (drivetrain == null || !drivetrain.isGyroConnected()) {
            faultArray.add("GyroDisconnected");
        }
        if (!joysticksConnected.getAsBoolean()) {
            faultArray.add("JoysticksDisconnected");
        }

        return faultArray;
    }

    public ArrayList<AbstractMap.SimpleEntry<Integer, SparkBase.Faults>> getSparkFaults() {
        ArrayList<AbstractMap.SimpleEntry<Integer, SparkBase.Faults>> sparkFaults = new ArrayList<>();
        for (var spark : this.sparks) {
            sparkFaults.add(
                    new AbstractMap.SimpleEntry<Integer, SparkBase.Faults>(
                            spark.getDeviceId(), spark.getFaults()));
        }
        return sparkFaults;
    }

    private String formatSparkFaults(
            ArrayList<AbstractMap.SimpleEntry<Integer, SparkBase.Faults>> sparkFaults) {
        final StringBuilder builder = new StringBuilder();
        for (var entry : sparkFaults) {
            if (entry.getValue().rawBits == 0) {
                continue;
            }
            builder.append(" { ID_" + entry.getKey() + ",");
            SparkBase.Faults faults = entry.getValue();
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
            builder.append(" },");
        }

        return builder.toString();
    }

    public void periodic() {
        if (faultUpdateTimer.hasElapsed(1)) {
            if (this.hasFaults()) {
                System.err.println(this.toString());
            }
            faultUpdateTimer.restart();
        }
        Logger.recordOutput("HasFaults", this.hasFaults());
        Logger.recordOutput("ClearForMatch", !this.hasFaults());
    }
}
