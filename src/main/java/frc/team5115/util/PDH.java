package frc.team5115.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import org.littletonrobotics.junction.Logger;

public class PDH {
    private final PowerDistribution pdh;
    private final boolean logPDH = false;

    public PDH() {
        pdh = logPDH ? new PowerDistribution(1, ModuleType.kRev) : null;
    }

    public void periodic() {
        if (logPDH) {
            Logger.recordOutput("PDH/ChannelCurrents", pdh.getAllCurrents());
            Logger.recordOutput("PDH/Temperature", pdh.getTemperature());
            Logger.recordOutput("PDH/TotalCurrent", pdh.getTotalCurrent());
            Logger.recordOutput("PDH/TotalEnergy", pdh.getTotalEnergy());
            Logger.recordOutput("PDH/TotalPower", pdh.getTotalPower());
            Logger.recordOutput("PDH/Voltage", pdh.getVoltage());
        }
    }
}
