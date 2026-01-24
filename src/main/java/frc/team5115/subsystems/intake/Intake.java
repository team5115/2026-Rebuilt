package frc.team5115.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import frc.team5115.util.MotorContainer;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements MotorContainer {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    private Command run(double speed) {
        return Commands.startEnd(() -> io.setPercent(speed), () -> io.setPercent(0), this);
    }

    public Command intake() {
        return run(Constants.INTAKE_SPEED);
    }

    public Command vomit() {
        return run(-1.0);
    }

    @Override
    public ArrayList<SparkMax> getSparks() {
        return io.getSparks();
    }

    public Command intakeIf(BooleanSupplier supplier) {
        return Commands.run(
                () -> {
                    if (supplier.getAsBoolean()) {
                        io.setPercent(Constants.INTAKE_SPEED);
                    } else {
                        io.setPercent(0);
                    }
                },
                this);
    }
}
