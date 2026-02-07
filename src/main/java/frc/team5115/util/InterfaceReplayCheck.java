package frc.team5115.util;

import frc.team5115.Constants;
import frc.team5115.Constants.Mode;

public class InterfaceReplayCheck {
    private InterfaceReplayCheck() {}

    public static void warnOnNotReplay() {
        if (Constants.currentMode != Mode.REPLAY) {
            StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();
            System.err.println(
                    "Error: Called unimplemented interface method "
                            + stackTrace[2].getMethodName()
                            + "@"
                            + stackTrace[2].getFileName()
                            + " from "
                            + stackTrace[3].getMethodName()
                            + "@"
                            + stackTrace[3].getFileName()
                            + ":"
                            + stackTrace[3].getLineNumber());
        }
    }
}
