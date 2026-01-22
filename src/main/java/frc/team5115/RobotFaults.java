package frc.team5115;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkMax;
import frc.team5115.subsystems.MotorContainer;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.vision.PhotonVision;
import java.util.ArrayList;

public class RobotFaults {
    // private static final String NO_FAULTS = "No Faults";
    public final Drivetrain drivetrain;
    public final PhotonVision vision;
    public final MotorContainer[] motorContainers;

    public final ArrayList<String> sparkFaults = new ArrayList<String>();
    public final ArrayList<String> faultArray = new ArrayList<String>();

    public RobotFaults(
            Drivetrain drivetrain, PhotonVision vision, MotorContainer... motorContainers) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.motorContainers = motorContainers;
    }

    // public String cacheString() {

    // }

    @Override
    public String toString() {
        return "SparkFaults:[ " + sparkFaults.toString() + "] ; " + faultArray.toString();
    }

    public boolean hasFaults() {
        return sparkFaults.size() > 0;
    }

    public void fromSubsystems(boolean joysticksConnected) {
        ArrayList<SparkMax> sparks = new ArrayList<>();
        if (vision == null || vision.areAnyCamerasDisconnected()) {
            faultArray.add("CameraDisconnected");
        }
        if (drivetrain == null || !drivetrain.isGyroConnected()) {
            faultArray.add("JoysticksDisconnected");
        }
        if (!joysticksConnected) {}

        if (drivetrain != null) {
            drivetrain.getSparks(sparks);
        }
        for (MotorContainer container : motorContainers) {
            if (container != null) {
                container.getSparks(sparks);
            }
        }

        sparkFaults.clear();
        for (var spark : sparks) {
            appendSparkFaults(spark.getFaults(), spark.getDeviceId());
        }
        ;
    }

    private void appendSparkFaults(Faults faults, int id) {

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
            sparkFaults.add("" + id);
            sparkFaults.add(",");
            sparkFaults.add(builder.toString());
            sparkFaults.add(" },");
        }
    }
}
