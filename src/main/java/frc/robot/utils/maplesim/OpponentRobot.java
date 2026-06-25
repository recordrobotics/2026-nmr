package frc.robot.utils.maplesim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.utils.ManagedSubsystemBase;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;

public class OpponentRobot extends ManagedSubsystemBase {

    private static final Set<OpponentRobot> allOpponentRobots = new HashSet<>();

    public record OpponentRobotState(Pose2d pose, int allianceStation) {}

    public static void logPoses() {
        List<OpponentRobotState> states = new ArrayList<>(allOpponentRobots.size());
        for (OpponentRobot robot : allOpponentRobots) {
            robot.getPose().ifPresent(pose -> states.add(new OpponentRobotState(pose, robot.getAllianceStation())));
        }

        Logger.recordOutput("OpponentRobot/States", states.toArray(new OpponentRobotState[0]));
    }

    @SuppressWarnings("EnumOrdinal") /* raw JNI api enum */
    public static void randomizeRobots() {
        int realStation = DriverStation.getRawAllianceStation().ordinal();
        List<Integer> possibleStations = new ArrayList<>();
        for (int i = 1; i <= 6; i++) {
            if (i != realStation) {
                possibleStations.add(i);
            }
        }

        possibleStations.sort((a, b) -> (int) (Math.random() * 3 - 1)); // Shuffle the possible stations

        for (OpponentRobot robot : allOpponentRobots) {
            int station = possibleStations.remove(0);
            robot.setAllianceStation(station);
            robot.enable();
        }
    }

    public enum Behavior {
        DEFENSE
    }

    private static final Pose2d[] ALLIANCE_START_POSITIONS = new Pose2d[] {
        Constants.FieldStartingLocation.TrenchDepot.getPose(Alliance.Red),
        Constants.FieldStartingLocation.Center.getPose(Alliance.Red),
        Constants.FieldStartingLocation.TrenchOutpost.getPose(Alliance.Red),
        Constants.FieldStartingLocation.TrenchDepot.getPose(Alliance.Blue),
        Constants.FieldStartingLocation.Center.getPose(Alliance.Blue),
        Constants.FieldStartingLocation.TrenchOutpost.getPose(Alliance.Blue)
    };

    private static Pose2d nextStartingPose = new Pose2d(-10, -10, Rotation2d.kZero);
    private static final Lock nextStartingPoseLock = new ReentrantLock(true);

    // Create and configure a drivetrain simulation configuration
    private final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
            // Specify gyro type (for realistic gyro drifting and error simulation)
            .withGyro(COTS.ofPigeon2())
            // Specify swerve module (for realistic swerve dynamics)
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                    DCMotor.getKrakenX44(1), // Steer motor is a Kraken X44
                    Constants.Swerve.KRAKEN_DRIVE_GEAR_RATIO, // Drive motor gear ratio.
                    Constants.Swerve.KRAKEN_TURN_GEAR_RATIO, // Steer motor gear ratio.
                    Volts.of(Constants.Swerve.KRAKEN_DRIVE_KS), // Drive static voltage
                    Volts.of(Constants.Swerve.KRAKEN_TURN_KS), // Steer static voltage
                    Meters.of(Constants.Swerve.WHEEL_DIAMETER / 2), // Wheel radius
                    KilogramSquareMeters.of(0.03),
                    COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof // Use the COF for Neoprene Tread
                    ))
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(
                    Meters.of(Constants.Frame.ROBOT_WHEEL_DISTANCE_LENGTH),
                    Meters.of(Constants.Frame.ROBOT_WHEEL_DISTANCE_WIDTH))
            // Configures the bumper size (dimensions of the robot bumper)
            .withBumperSize(
                    Meters.of(Constants.Frame.FRAME_WITH_BUMPER_LENGTH),
                    Meters.of(Constants.Frame.FRAME_WITH_BUMPER_WIDTH))
            .withRobotMass(Kilograms.of(Constants.Frame.ROBOT_MASS_KG));

    private final SelfControlledSwerveDriveSimulation driveSimulation;
    private final Pose2d startingPose;

    private boolean enabled = false;
    private int allianceStation = 1;
    private Behavior behavior = Behavior.DEFENSE;

    public static OpponentRobot create() {
        if (Constants.RobotState.getMode() != Mode.SIM) {
            throw new IllegalStateException("OpponentRobot should only be instantiated in simulation mode!");
        }

        Pose2d startingPose = Pose2d.kZero;

        nextStartingPoseLock.lock();
        try {
            startingPose = nextStartingPose;
            nextStartingPose = nextStartingPose.plus(new Transform2d(-2, 0, Rotation2d.kZero));
        } finally {
            nextStartingPoseLock.unlock();
        }

        OpponentRobot robot = new OpponentRobot(startingPose);
        allOpponentRobots.add(robot);
        return robot;
    }

    private OpponentRobot(Pose2d startingPose) {
        if (Constants.RobotState.getMode() != Mode.SIM) {
            throw new IllegalStateException("OpponentRobot should only be instantiated in simulation mode!");
        }

        this.startingPose = startingPose;

        driveSimulation = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(driveTrainSimulationConfig, startingPose));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
    }

    public Optional<Pose2d> getPose() {
        if (!enabled) {
            return Optional.empty();
        }

        return Optional.of(driveSimulation.getActualPoseInSimulationWorld());
    }

    public void enable() {
        driveSimulation.setSimulationWorldPose(ALLIANCE_START_POSITIONS[allianceStation - 1]);
        enabled = true;
    }

    public void disable() {
        enabled = false;
        driveSimulation.setSimulationWorldPose(startingPose);
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setAllianceStation(int station) {
        if (station < 1 || station > 6) {
            throw new IllegalArgumentException("Alliance station must be between 1 and 6");
        }

        allianceStation = station;

        if (enabled) {
            driveSimulation.setSimulationWorldPose(ALLIANCE_START_POSITIONS[allianceStation - 1]);
        }
    }

    public int getAllianceStation() {
        return allianceStation;
    }

    public void setBehavior(Behavior behavior) {
        this.behavior = behavior;
    }

    public Behavior getBehavior() {
        return behavior;
    }
}
