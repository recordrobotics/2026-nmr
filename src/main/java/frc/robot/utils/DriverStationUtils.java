package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.OptionalDouble;

public final class DriverStationUtils {

    private static Command practiceModeDetectionCommand;

    private static boolean inPracticeMode = false;
    private static double teleopInitMatchTime = 0;

    private DriverStationUtils() {}

    static {
        practiceModeDetectionCommand = getPracticeModeDetectionCommand();
    }

    public static Alliance getCurrentAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get();
        }
        return Alliance.Blue;
    }

    @SuppressWarnings("java:S1244") // matchTime is exactly -1
    public static void teleopInit() {
        inPracticeMode = false;

        // Practice mode detection is only without FMS
        if (DriverStation.isFMSAttached()) return;

        teleopInitMatchTime = DriverStation.getMatchTime();
        practiceModeDetectionCommand = getPracticeModeDetectionCommand();
        CommandScheduler.getInstance().schedule(practiceModeDetectionCommand);
    }

    private static Command getPracticeModeDetectionCommand() {
        return Commands.waitUntil(DriverStationUtils::practiceModeDetection);
    }

    private static boolean practiceModeDetection() {
        double matchTime = DriverStation.getMatchTime();

        if (matchTime == -1) {
            // Match time not valid, can't be in practice mode
            return true;
        }

        if (Math.abs(teleopInitMatchTime - matchTime) < 1e-4) return false; // Wait for update

        // After update, make sure the match time goes down instead of up
        if (matchTime < teleopInitMatchTime) {
            inPracticeMode = true;
        }

        return true;
    }

    public static boolean isInPracticeMode() {
        return inPracticeMode;
    }

    public enum MatchShift {
        AUTO("Auto"),
        TRANSITION("Transition"),
        SHIFT_1("Shift 1"),
        SHIFT_2("Shift 2"),
        SHIFT_3("Shift 3"),
        SHIFT_4("Shift 4"),
        ENDGAME("Endgame"),
        UNKNOWN("Unknown");

        private final String name;

        MatchShift(String name) {
            this.name = name;
        }

        public String getName() {
            return name;
        }
    }

    public record MatchTimeData(
            boolean previousHubActive,
            boolean currentHubActive,
            boolean nextHubActive,
            double timeSinceShift,
            double timeLeftInShift,
            MatchShift shift,
            boolean wonAuto) {}

    public static MatchTimeData getMatchTimeData() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return new MatchTimeData(false, false, false, 0, 0, MatchShift.UNKNOWN, false);
        }

        double matchTime = DriverStation.getMatchTime();

        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return new MatchTimeData(false, true, true, 20 - matchTime, matchTime, MatchShift.AUTO, false);
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return new MatchTimeData(false, false, false, 0, 25, MatchShift.UNKNOWN, false);
        }

        // We're teleop enabled, compute.
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return new MatchTimeData(true, true, true, 0, 0, MatchShift.UNKNOWN, false);
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return new MatchTimeData(true, true, true, 0, 0, MatchShift.UNKNOWN, false);
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = alliance.get() == Alliance.Red ? !redInactiveFirst : redInactiveFirst;

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return new MatchTimeData(
                    true, true, shift1Active, 140 - matchTime, matchTime - 130, MatchShift.TRANSITION, !shift1Active);
        } else if (matchTime > 105) {
            // Shift 1
            return new MatchTimeData(
                    true,
                    shift1Active,
                    !shift1Active,
                    130 - matchTime,
                    matchTime - 105,
                    MatchShift.SHIFT_1,
                    !shift1Active);
        } else if (matchTime > 80) {
            // Shift 2
            return new MatchTimeData(
                    shift1Active,
                    !shift1Active,
                    shift1Active,
                    105 - matchTime,
                    matchTime - 80,
                    MatchShift.SHIFT_2,
                    !shift1Active);
        } else if (matchTime > 55) {
            // Shift 3
            return new MatchTimeData(
                    !shift1Active,
                    shift1Active,
                    !shift1Active,
                    80 - matchTime,
                    matchTime - 55,
                    MatchShift.SHIFT_3,
                    !shift1Active);
        } else if (matchTime > 30) {
            // Shift 4
            return new MatchTimeData(
                    shift1Active,
                    !shift1Active,
                    true,
                    55 - matchTime,
                    matchTime - 30,
                    MatchShift.SHIFT_4,
                    !shift1Active);
        } else {
            // End game, hub always active.
            return new MatchTimeData(
                    !shift1Active, true, false, 30 - matchTime, matchTime, MatchShift.ENDGAME, !shift1Active);
        }
    }

    /**
     * Returns the time remaining in the TELEOP period (in rounded INTEGER seconds)
     * <hr>
     * <p>
     * If the current period is not teleop, returns empty
     * </p>
     * <p>
     * If NOT connected to FMS (driver practice):
     * <ul>
     *     <li>If in practice mode returns time remaining as usual</li>
     *     <li>If NOT in practice mode returns empty
     *            (no defined remaining match time since
     *            could be enabled for any amount of time)</li>
     * </ul>
     * </p>
     * @return the remaining match time in the TELEOP period
     * (in rounded INTEGER seconds) or empty if undefined
     */
    public static OptionalDouble getTeleopMatchTime() {
        if (!DriverStation.isTeleop()) return OptionalDouble.empty(); // Not in teleop

        if (!DriverStation.isFMSAttached() && !isInPracticeMode())
            return OptionalDouble.empty(); // If no fms, empty if not in practice mode

        // FMS is attached or in practice mode
        double matchTime = DriverStation.getMatchTime();
        return OptionalDouble.of(matchTime);
    }
}
