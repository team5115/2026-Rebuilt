package frc.team5115.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.team5115.Constants;

public class ClimberIOSim implements ClimberIO {
    private final DCMotorSim sim;
    private double appliedVolts;

    public ClimberIOSim() {
        DCMotor motor = DCMotor.getNEO(1);
        sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.2, 1.0), motor);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        sim.update(Constants.LOOP_PERIOD_SECS);
        inputs.velocityRPM = sim.getAngularVelocityRPM();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
        inputs.positionRad = sim.getAngularPositionRad();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, +12.0);
        sim.setInputVoltage(appliedVolts);
    }
}
