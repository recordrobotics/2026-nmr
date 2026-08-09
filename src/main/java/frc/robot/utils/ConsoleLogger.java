package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public final class ConsoleLogger {

    private static final int TRACE_START_INDEX = 3;
    private static final String TRACE_SEPARATOR = "\n    -----------\n";

    private static final String INFO = "INFO";
    private static final String WARNING = "WARNING";
    private static final String ERROR = "ERROR";

    private ConsoleLogger() {}

    public static void log(String type, String message) {
        logImpl(type, message, 2);
    }

    public static void logInfo(String message) {
        logImpl(INFO, message, TRACE_START_INDEX);
    }

    public static void logWarning(String message) {
        logImpl(WARNING, message, TRACE_START_INDEX);
    }

    public static void logError(String message) {
        logImpl(ERROR, message, TRACE_START_INDEX);
    }

    public static void log(String type, Throwable e) {
        logImpl(type, e, 2);
    }

    public static void log(String type, String message, Throwable e) {
        logImpl(type, message, e, 2);
    }

    public static void logInfo(Throwable e) {
        logImpl(INFO, e, TRACE_START_INDEX);
    }

    public static void logInfo(String message, Throwable e) {
        logImpl(INFO, message, e, TRACE_START_INDEX);
    }

    public static void logWarning(Throwable e) {
        logImpl(WARNING, e, TRACE_START_INDEX);
    }

    public static void logWarning(String message, Throwable e) {
        logImpl(WARNING, message, e, TRACE_START_INDEX);
    }

    public static void logError(Throwable e) {
        logImpl(ERROR, e, TRACE_START_INDEX);
    }

    public static void logError(String message, Throwable e) {
        logImpl(ERROR, message, e, TRACE_START_INDEX);
    }

    private static String trace(StackTraceElement[] stackTrace, int traceStartIndex) {
        StringBuilder traceBuilder = new StringBuilder();
        for (int i = traceStartIndex; i < stackTrace.length; i++) {
            traceBuilder.append("    at ");
            traceBuilder.append(stackTrace[i].toString());
            if (i < stackTrace.length - 1) {
                traceBuilder.append("\n");
            }
        }
        return traceBuilder.toString();
    }

    private static void logImpl(String type, String message, int traceStartIndex) {
        StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();
        String className = stackTrace[traceStartIndex].getClassName();
        String text =
                "[" + type + " / " + className + "]: " + message + TRACE_SEPARATOR + trace(stackTrace, traceStartIndex);

        // Only report to DriverStation on real robot (no access to console on field)
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL) {
            // Ends up printing to console twice, but ensures both AdvantageKit log and the driver station have the
            // message
            if (type.equals(ERROR)) {
                DriverStation.reportError(text, false);
            } else if (type.equals(WARNING)) {
                DriverStation.reportWarning(text, false);
            }
        }

        System.out.println(text);
    }

    private static void logImpl(String type, Throwable e, int traceStartIndex) {
        logImpl(type, "Exception: " + e.getMessage() + TRACE_SEPARATOR + trace(e.getStackTrace(), 0), traceStartIndex);
    }

    private static void logImpl(String type, String message, Throwable e, int traceStartIndex) {
        logImpl(
                type,
                message + "\n    Exception: " + e.getMessage() + TRACE_SEPARATOR + trace(e.getStackTrace(), 0),
                traceStartIndex);
    }
}
