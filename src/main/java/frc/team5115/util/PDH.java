package frc.team5115.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import org.littletonrobotics.junction.Logger;

public class PDH {
    private final PowerDistribution pdh;

    public PDH() {
        pdh = new PowerDistribution(1, ModuleType.kRev);
    }

    public void periodic() {
        Logger.recordOutput("PDH/ChannelCurrents", pdh.getAllCurrents());
        Logger.recordOutput("PDH/Temperature", pdh.getTemperature());
        Logger.recordOutput("PDH/TotalCurrent", pdh.getTotalCurrent());
        Logger.recordOutput("PDH/TotalEnergy", pdh.getTotalEnergy());
        Logger.recordOutput("PDH/TotalPower", pdh.getTotalPower());
        Logger.recordOutput("PDH/Voltage", pdh.getVoltage());
    }
}
