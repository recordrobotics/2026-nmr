package frc.robot.commands.auto;

public class Auto {
    private Auto() {
        /* This utility class should not be instantiated */
    }

    public static class AutoLoadException extends RuntimeException {
        public AutoLoadException(String message, Throwable cause) {
            super(message, cause);
        }
    }
}
