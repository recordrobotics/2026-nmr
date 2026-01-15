package frc.robot.utils.maplesim;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.SimpleMath;
import java.util.Random;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

public class ImprovedMapleMatch {

    private static final Random RANDOM = new Random();

    // Since maplesim counts scoring from the entrance of fuel into the funnel, account for the time it takes to get
    // from the funnel to the counter.
    private static final double HUB_FUNNEL_TO_COUNTER_TIME_SECONDS = 1.0;
    // Since maplesim counts scoring from the entrance of fuel into the funnel, subtract the time it takes to get from
    // the funnel to the counter from the 3 second active time.
    private static final double COUNTER_ACTIVE_AFTER_SHIFT_SECONDS = 3.0 - HUB_FUNNEL_TO_COUNTER_TIME_SECONDS;

    private static ImprovedMapleMatch instance;

    public static ImprovedMapleMatch getInstance() {
        if (instance == null) {
            instance = new ImprovedMapleMatch();
        }
        return instance;
    }

    private double oldRedFuel = 0;
    private double oldBlueFuel = 0;
    private double redFuelWhileActive = 0;
    private double blueFuelWhileActive = 0;
    private boolean hasTeleopStarted = false;
    private boolean redInactiveFirst = false;

    private boolean redActive = false;
    private boolean blueActive = false;

    private boolean redCounterActive = false;
    private boolean blueCounterActive = false;

    private double shiftEndTime = 0;
    private boolean nextShiftRedActive = false;
    private boolean nextShiftBlueActive = false;

    private double nextThrowInTimestamp = Timer.getTimestamp();

    private double disabledTimestamp = 0;

    public ImprovedMapleMatch() {
        sendActiveState();
        sendScore();
        DriverStationSim.notifyNewData();
    }

    public void sendActiveState() {
        double timeLeftInShift = shiftEndTime - Timer.getTimestamp();
        boolean flash = hasTeleopStarted
                && timeLeftInShift < 3
                && (timeLeftInShift % 0.5 < 0.25); // Flash in the last 3 seconds of a shift

        SmartDashboard.putBoolean("MapleSim/MatchData/Breakdown/Red Alliance/Improved Active", redActive);
        SmartDashboard.putBoolean("MapleSim/MatchData/Breakdown/Blue Alliance/Improved Active", blueActive);
        SmartDashboard.putBoolean(
                "MapleSim/MatchData/Breakdown/Red Alliance/Improved Hub Led",
                redActive && (!flash || nextShiftRedActive));
        SmartDashboard.putBoolean(
                "MapleSim/MatchData/Breakdown/Blue Alliance/Improved Hub Led",
                blueActive && (!flash || nextShiftBlueActive));
    }

    public void periodic() {
        updateActiveStates();

        updateHubCounters();
        sendActiveState();
        sendScore();

        throwFuelBackIntoField();
    }

    private void updateHubCounters() {
        double redFuel = SmartDashboard.getNumber("MapleSim/MatchData/Breakdown/Red Alliance/TotalFuelInHub", 0);
        double blueFuel = SmartDashboard.getNumber(
                "MapleSim/MatchData/Breakdown/blue Alliance/TotalFuelInHub", 0); // maplesim typo

        if (redFuel > oldRedFuel) {
            if (redCounterActive) {
                redFuelWhileActive += redFuel - oldRedFuel;
            }
            oldRedFuel = redFuel;
        } else if (redFuel < oldRedFuel) {
            oldRedFuel = redFuel;
        }

        if (blueFuel > oldBlueFuel) {
            if (blueCounterActive) {
                blueFuelWhileActive += blueFuel - oldBlueFuel;
            }
            oldBlueFuel = blueFuel;
        } else if (blueFuel < oldBlueFuel) {
            oldBlueFuel = blueFuel;
        }
    }

    private void throwFuelBackIntoField() {
        // Make sure balls get rolled back
        // Check if ball is OOB
        GamePieceOnFieldSimulation[] fuelPoses =
                SimulatedArena.getInstance().gamePiecesOnField().toArray(GamePieceOnFieldSimulation[]::new);

        if (Timer.getTimestamp() - nextThrowInTimestamp > 0) {
            GamePieceOnFieldSimulation fuel = fuelPoses[RANDOM.nextInt(0, fuelPoses.length - 1)];
            if (!SimpleMath.isInField(fuel.getPoseOnField())) {

                // x is up and down
                // y is left and right

                Translation2d currentPos = fuel.getPose3d().getTranslation().toTranslation2d();

                double randomAdd = Math.random() * 2.5;

                double newX = currentPos.getX();
                double newY = currentPos.getY();

                double newXVelocity = 0.0;
                double newYVelocity = 0.0;

                // top and bottom checks
                if (currentPos.getX() < FlippingUtil.fieldSizeX) {
                    newX = 0.25;
                    newXVelocity = 3.0 + randomAdd;
                } else if (currentPos.getX() > FlippingUtil.fieldSizeX) {
                    newX = FlippingUtil.fieldSizeX - 0.25;
                    newXVelocity = -3.0 - randomAdd;
                }

                // left and right checks
                if (currentPos.getY() < FlippingUtil.fieldSizeY) {
                    newY = 0.25;
                    newYVelocity = 3.0 + randomAdd;
                } else if (currentPos.getY() > FlippingUtil.fieldSizeY) {
                    newY = FlippingUtil.fieldSizeY - 0.25;
                    newYVelocity = -3.0 - randomAdd;
                }

                SimulatedArena.getInstance().removeGamePiece(fuel); // delete fuel

                SimulatedArena.getInstance() // spawn fuel
                        .addGamePieceProjectile(new GamePieceProjectile(
                                        RebuiltFuelOnField.REBUILT_FUEL_INFO,
                                        new Translation2d(newX, newY), // start pos (x, y)
                                        new Translation2d( // start velocity
                                                newXVelocity, newYVelocity),
                                        2 + Math.random() * 1.5, // initial height
                                        0 + Math.random(), // vertical speed
                                        new Rotation3d(0, 0, 0)) // rotation
                                .enableBecomesGamePieceOnFieldAfterTouchGround());

                nextThrowInTimestamp = Timer.getTimestamp() + Math.random() * 1.5 + 0.5;
            }
        }
    }

    public void autonomousInit() {
        redFuelWhileActive = 0;
        blueFuelWhileActive = 0;
        hasTeleopStarted = false;
        nextShiftRedActive = false;
        nextShiftBlueActive = false;

        DriverStationSim.setGameSpecificMessage("");
        DriverStationSim.notifyNewData();
        redActive = true;
        blueActive = true;
        redCounterActive = true;
        blueCounterActive = true;
        sendActiveState();
        sendScore();
    }

    public void teleopInit() {
        hasTeleopStarted = true;

        if (oldRedFuel > oldBlueFuel) {
            redInactiveFirst = true;
        } else if (oldBlueFuel > oldRedFuel) {
            redInactiveFirst = false;
        } else {
            redInactiveFirst = RANDOM.nextBoolean();
        }

        DriverStationSim.setGameSpecificMessage(redInactiveFirst ? "R" : "B");
        DriverStationSim.notifyNewData();
    }

    public void disabledInit() {
        disabledTimestamp = Timer.getTimestamp();
    }

    public void updateActiveStates() {
        if (!hasTeleopStarted) {
            redActive = true;
            blueActive = true;
            redCounterActive = true;
            blueCounterActive = true;
        } else if (DriverStation.isTeleopEnabled()) {
            double matchTime = Math.floor(DriverStation.getMatchTime());

            if (matchTime > 130) {
                // Transition shift, hub is active.
                redActive = true;
                blueActive = true;
                if (matchTime >= 130 + 3) {
                    nextShiftRedActive = !redInactiveFirst;
                    nextShiftBlueActive = redInactiveFirst;
                    shiftEndTime = Timer.getTimestamp() + 3.1;
                }
            } else if (matchTime > 105) {
                // Shift 1
                redActive = !redInactiveFirst;
                blueActive = redInactiveFirst;

                redCounterActive = redActive
                        || matchTime > 130 - COUNTER_ACTIVE_AFTER_SHIFT_SECONDS
                        || matchTime < 105 + HUB_FUNNEL_TO_COUNTER_TIME_SECONDS;
                blueCounterActive = blueActive
                        || matchTime > 130 - COUNTER_ACTIVE_AFTER_SHIFT_SECONDS
                        || matchTime < 105 + HUB_FUNNEL_TO_COUNTER_TIME_SECONDS;

                if (matchTime >= 105 + 3) {
                    nextShiftRedActive = redInactiveFirst;
                    nextShiftBlueActive = !redInactiveFirst;
                    shiftEndTime = Timer.getTimestamp() + 3.1;
                }
            } else if (matchTime > 80) {
                // Shift 2
                redActive = redInactiveFirst;
                blueActive = !redInactiveFirst;

                redCounterActive = redActive
                        || matchTime > 105 - COUNTER_ACTIVE_AFTER_SHIFT_SECONDS
                        || matchTime < 80 + HUB_FUNNEL_TO_COUNTER_TIME_SECONDS;
                blueCounterActive = blueActive
                        || matchTime > 105 - COUNTER_ACTIVE_AFTER_SHIFT_SECONDS
                        || matchTime < 80 + HUB_FUNNEL_TO_COUNTER_TIME_SECONDS;

                if (matchTime >= 80 + 3) {
                    nextShiftRedActive = !redInactiveFirst;
                    nextShiftBlueActive = redInactiveFirst;
                    shiftEndTime = Timer.getTimestamp() + 3.1;
                }
            } else if (matchTime > 55) {
                // Shift 3
                redActive = !redInactiveFirst;
                blueActive = redInactiveFirst;

                redCounterActive = redActive
                        || matchTime > 80 - COUNTER_ACTIVE_AFTER_SHIFT_SECONDS
                        || matchTime < 55 + HUB_FUNNEL_TO_COUNTER_TIME_SECONDS;
                blueCounterActive = blueActive
                        || matchTime > 80 - COUNTER_ACTIVE_AFTER_SHIFT_SECONDS
                        || matchTime < 55 + HUB_FUNNEL_TO_COUNTER_TIME_SECONDS;

                if (matchTime >= 55 + 3) {
                    nextShiftRedActive = redInactiveFirst;
                    nextShiftBlueActive = !redInactiveFirst;
                    shiftEndTime = Timer.getTimestamp() + 3.1;
                }
            } else if (matchTime > 30) {
                // Shift 4
                redActive = redInactiveFirst;
                blueActive = !redInactiveFirst;

                redCounterActive = redActive
                        || matchTime > 55 - COUNTER_ACTIVE_AFTER_SHIFT_SECONDS
                        || matchTime < 30 + HUB_FUNNEL_TO_COUNTER_TIME_SECONDS;
                blueCounterActive = blueActive
                        || matchTime > 55 - COUNTER_ACTIVE_AFTER_SHIFT_SECONDS
                        || matchTime < 30 + HUB_FUNNEL_TO_COUNTER_TIME_SECONDS;

                if (matchTime >= 30 + 3) {
                    nextShiftRedActive = true;
                    nextShiftBlueActive = true;
                    shiftEndTime = Timer.getTimestamp() + 3.1;
                }
            } else {
                // End game, hub always active.
                redActive = true;
                blueActive = true;

                redCounterActive = true;
                blueCounterActive = true;

                if (matchTime >= 3) {
                    nextShiftRedActive = false;
                    nextShiftBlueActive = false;
                    shiftEndTime = Timer.getTimestamp() + 3.1;
                } else if (matchTime < 0) {
                    // Not a practice match
                    nextShiftRedActive = true;
                    nextShiftBlueActive = true;
                }
            }
        } else {
            redActive = false;
            blueActive = false;

            double timeSinceDisabled = Timer.getTimestamp() - disabledTimestamp;
            redCounterActive = timeSinceDisabled < COUNTER_ACTIVE_AFTER_SHIFT_SECONDS;
            blueCounterActive = timeSinceDisabled < COUNTER_ACTIVE_AFTER_SHIFT_SECONDS;
        }
    }

    public void sendScore() {
        SmartDashboard.putNumber("MapleSim/MatchData/Breakdown/Red Alliance/Improved Score", redFuelWhileActive);
        SmartDashboard.putNumber("MapleSim/MatchData/Breakdown/Blue Alliance/Improved Score", blueFuelWhileActive);
    }
}
