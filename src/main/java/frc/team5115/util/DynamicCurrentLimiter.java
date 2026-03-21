package frc.team5115.util;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.IntConsumer;

public class DynamicCurrentLimiter {
    private final int maxLimit;
    private final int minLimit;
    private final int stepSize;
    private final Trigger stepDownTrigger;
    private final IntConsumer onCurrentLimitChange;
    private int dynamicLimit;
    private int overrideLimit = -1;

    public DynamicCurrentLimiter(
            int maxLimit,
            int minLimit,
            int stepSize,
            double debounceTime,
            IntConsumer onCurrentLimitChange) {
        this.maxLimit = maxLimit;
        this.minLimit = minLimit;
        this.stepSize = stepSize;
        this.onCurrentLimitChange = onCurrentLimitChange;
        dynamicLimit = maxLimit;
        stepDownTrigger =
                new Trigger(RobotController::isBrownedOut)
                        .and(() -> overrideLimit > -1)
                        .debounce(debounceTime, DebounceType.kFalling);
        stepDownTrigger.onTrue(Commands.runOnce(this::stepDown));
    }

    public void stepDown() {
        setDynamicLimit(Math.max(minLimit, dynamicLimit - stepSize));
    }

    private void setDynamicLimit(int newLimit) {
        final int oldLimit = getCurrentLimit();
        dynamicLimit = newLimit;
        if (newLimit != oldLimit) {
            notifySubscribers();
        }
    }

    public void resetToMax() {
        setDynamicLimit(maxLimit);
    }

    public Command commandOverrideLimit(int limit) {
        return Commands.startEnd(
                () -> {
                    overrideLimit = limit;
                    notifySubscribers();
                },
                () -> {
                    overrideLimit = -1;
                    notifySubscribers();
                });
    }

    public void hardOverrideLimit(int limit) {
        overrideLimit = limit;
        notifySubscribers();
    }

    public void clearOverride() {
        hardOverrideLimit(-1);
    }

    private void notifySubscribers() {
        onCurrentLimitChange.accept(getCurrentLimit());
    }

    public int getCurrentLimit() {
        return overrideLimit > -1 ? overrideLimit : dynamicLimit;
    }

    public boolean steppingDown() {
        return stepDownTrigger.getAsBoolean();
    }

    public int getOverride() {
        return overrideLimit;
    }
}
