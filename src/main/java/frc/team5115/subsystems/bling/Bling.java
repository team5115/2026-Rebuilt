package frc.team5115.subsystems.bling;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Bling extends SubsystemBase {
    public static final int LED_COUNT = 144; // ! must be divisble by 3

    private static final DoubleSupplier zeros = () -> 0;
    private static final DoubleSupplier redAllianceSupplier = () -> Constants.isRedAlliance() ? 1 : 0;
    private static final DoubleSupplier blueAllianceSupplier =
            () -> Constants.isRedAlliance() ? 0 : 1;

    private final BlingIO io;
    private final BlingIOInputsAutoLogged inputs = new BlingIOInputsAutoLogged();

    private final int period = 1;
    private final double tailLength = 20;
    private final int minPower = 10;
    private final int maxPower = 255;
    private final double decay = (maxPower - minPower) / 1d / tailLength;

    private int timer = 0;
    private int counter = 0;
    private int direction = 1;
    private int counter2 = 0;
    int on = 1;

    public Bling(BlingIO io) {
        this.io = io;
        io.start();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Bling/LedStrip", inputs.ledStrip);
        Logger.recordOutput("Bling/Timer", timer);
        Logger.recordOutput("Bling/Counter", counter);
        Logger.recordOutput("Bling/Direction", direction);
    }

    /** Set the whole strip to off */
    public Command off() {
        return staticColor(0, 0, 0, 0);
    }

    /** The classic scrolling KITT pattern but based on alliance color */
    public Command allianceKITT() {
        return scrollingKITT(redAllianceSupplier, zeros, blueAllianceSupplier, zeros);
    }

    /** Scroll in with color based on alliance color */
    public Command allianceScrollIn() {
        return scrollIn(redAllianceSupplier, zeros, blueAllianceSupplier, zeros);
    }

    public Command whiteScrollIn() {
        return scrollIn(zeros, zeros, zeros, () -> 1);
    }

    public Command blueSolid() {
        return staticColor(0, 0, 1, 0);
    }

    public Command purpleSolid() {
        return staticColor(0.35f, 0, 0.75f, 0);
    }

    public Command purpleFlashing() {
        return seizure(() -> 0.35f, zeros, () -> 0.75f, zeros, () -> 0.35f, zeros, () -> 0.75f, zeros);
    }

    public Command allianceWhiteFlashing() {
        return seizure(
                zeros, zeros, zeros, () -> 0.5, redAllianceSupplier, zeros, blueAllianceSupplier, zeros);
    }

    public Command purpleScrollIn() {
        return scrollIn(() -> 0.35f, zeros, () -> 0.75f, zeros);
    }

    public Command faultFlash() {
        return seizure(() -> 0.1, zeros, zeros, zeros, () -> 0.1, () -> 0.05, zeros, zeros);
    }

    /**
     * Set the whole strip to a color
     *
     * @param red the red component (0-255)
     * @param green the green component (0-255)
     * @param blue the blue component (0-255)
     * @param white the white component (0-255)
     * @return a command that sets the color
     */
    public Command staticColor(float red, float green, float blue, float white) {
        return Commands.startRun(
                () -> {},
                () -> {
                    for (int i = 0; i < LED_COUNT; i++) {
                        io.setRGBW(
                                i, (int) (red * 255), (int) (green * 255), (int) (blue * 255), (int) (white * 255));
                    }
                },
                this);
    }

    /**
     * The classic KITT light pattern using the color percentages passed in
     *
     * @param redSupplier [0,1]
     * @param greenSupplier [0,1]
     * @param blueSupplier [0,1]
     * @param whiteSupplier [0,1]
     * @return a command that runs the pattern
     */
    public Command scrollingKITT(
            DoubleSupplier redSupplier,
            DoubleSupplier greenSupplier,
            DoubleSupplier blueSupplier,
            DoubleSupplier whiteSupplier) {
        return Commands.startRun(
                () -> {
                    // Start
                    for (int i = 0; i < LED_COUNT; i++) {
                        io.setRGBW(i, 0, 0, 0, 0);
                    }
                },
                () -> {
                    // Repeating
                    double red = redSupplier.getAsDouble();
                    double blue = blueSupplier.getAsDouble();
                    double green = greenSupplier.getAsDouble();
                    double white = whiteSupplier.getAsDouble();

                    timer++;
                    if (timer >= period) {
                        timer = 0;
                    } else {
                        return;
                    }

                    counter += direction;
                    if (counter >= LED_COUNT || counter < 0) {
                        direction = -direction;
                        counter = direction > 0 ? 0 : LED_COUNT - 1;
                    }

                    for (int i = 0; i < LED_COUNT; i++) {
                        double power;
                        if (i == counter) {
                            power = maxPower;
                        } else {
                            power =
                                    Math.max(
                                            minPower,
                                            totalBrightness(inputs.ledStrip[i]) / (red + green + blue + white) - decay);
                        }
                        io.setRGBW(
                                i,
                                (int) (power * red),
                                (int) (power * green),
                                (int) (power * blue),
                                (int) (power * white));
                    }
                },
                this);
    }

    public Command seizure(
            DoubleSupplier redSupplier,
            DoubleSupplier greenSupplier,
            DoubleSupplier blueSupplier,
            DoubleSupplier whiteSupplier,
            DoubleSupplier red2Supplier,
            DoubleSupplier green2Supplier,
            DoubleSupplier blue2Supplier,
            DoubleSupplier white2Supplier) {
        final double adjPeriod = (period / 20 / 4);
        return Commands.startRun(
                () -> {
                    // Start
                    for (int i = 0; i < LED_COUNT; i++) {
                        io.setRGBW(i, 0, 0, 0, 0);
                    }
                },
                () -> {
                    // Repeating
                    final double red = redSupplier.getAsDouble();
                    final double green = greenSupplier.getAsDouble();
                    final double blue = blueSupplier.getAsDouble();
                    final double white = whiteSupplier.getAsDouble();
                    final double red2 = red2Supplier.getAsDouble();
                    final double green2 = green2Supplier.getAsDouble();
                    final double blue2 = blue2Supplier.getAsDouble();
                    final double white2 = white2Supplier.getAsDouble();
                    timer++;
                    if (timer >= adjPeriod) {
                        timer = 0;
                        on = on == 1 ? 0 : 1;
                    } else {
                        return;
                    }
                    if (on == 1) {
                        for (int i = 0; i < LED_COUNT; i++) {
                            io.setRGBW(
                                    i,
                                    (int) (red * 255),
                                    (int) (green * 255),
                                    (int) (blue * 255),
                                    (int) (white * 255));
                        }
                    } else {
                        for (int i = 0; i < LED_COUNT; i++) {
                            io.setRGBW(
                                    i,
                                    (int) (red2 * 255),
                                    (int) (green2 * 255),
                                    (int) (blue2 * 255),
                                    (int) (white2 * 255));
                        }
                    }
                },
                this);
    }

    public Command scrollIn(
            DoubleSupplier redSupplier,
            DoubleSupplier greenSupplier,
            DoubleSupplier blueSupplier,
            DoubleSupplier whiteSupplier) {
        return Commands.startRun(
                () -> {
                    // Start
                    for (int i = 0; i < LED_COUNT; i++) {
                        io.setRGBW(i, 0, 0, 0, 0);
                    }
                    counter2 = LED_COUNT;
                    counter = 0;
                },
                () -> {
                    // Repeating
                    final double red = redSupplier.getAsDouble();
                    final double green = greenSupplier.getAsDouble();
                    final double blue = blueSupplier.getAsDouble();
                    final double white = whiteSupplier.getAsDouble();
                    timer++;
                    if (timer >= period) {
                        timer = 0;
                    } else {
                        return;
                    }

                    counter += 1;
                    counter2 += -1;
                    if (counter >= LED_COUNT - LED_COUNT / 2) {
                        counter = 0;
                        counter2 = LED_COUNT;
                    }

                    for (int i = 0; i < LED_COUNT; i++) {
                        double power;
                        if (i == counter || i == counter2) {
                            power = maxPower;
                        } else {
                            power =
                                    Math.max(
                                            minPower,
                                            totalBrightness(inputs.ledStrip[i]) / (red + green + blue + white) - decay);
                        }
                        io.setRGBW(
                                i,
                                (int) (power * red),
                                (int) (power * green),
                                (int) (power * blue),
                                (int) (power * white));
                    }
                },
                this);
    }

    private double totalBrightness(int[] color) {
        return color[0] + color[1] + color[2] + color[3];
    }
}
