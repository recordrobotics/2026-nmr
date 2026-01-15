package frc.robot.utils.maplesim;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.libraries.bumpsim.RobotBumpSim;
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
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.Logger;

public class OpponentRobot extends ManagedSubsystemBase {

    private static final Set<OpponentRobot> allOpponentRobots = new HashSet<>();

    public static void logPoses() {
        for (OpponentRobot robot : allOpponentRobots) {
            String prefix = "OpponentRobot/" + robot.robotId + "/";
            if (robot.isEnabled()) {
                Logger.recordOutput(prefix + "Pose", robot.getPose());
                Logger.recordOutput(prefix + "AllianceStation", robot.allianceStation);
            }

            Logger.recordOutput(prefix + "Enabled", robot.isEnabled());
        }
    }

    public static void createDefaultOpponents() {
        for (int i = 0; i < 6; i++) {
            String key = "opponent." + (i < 3 ? "red" : "blue") + (i % 3 + 1);
            String robotData = System.getProperty(key, "");
            Optional<Behavior> behavior = Behavior.fromPropertyName(robotData);
            if (behavior.isPresent()) {
                OpponentRobot robot = OpponentRobot.create();
                robot.setAllianceStation(i + 1);
                robot.setBehavior(behavior.get());
            }
        }
    }

    public static void resetAndEnableAllOthers() {
        int station = DriverStation.getRawAllianceStation().ordinal();

        for (OpponentRobot robot : allOpponentRobots) {
            if (robot.getAllianceStation() != station) {
                robot.resetAndEnable();
            } else {
                robot.disable();
            }
        }
    }

    public static List<Pair<Translation2d, Translation2d>> getAllOpponentRobotBoundingBoxes(OpponentRobot self) {
        List<Pair<Translation2d, Translation2d>> boundingBoxes = new ArrayList<>();
        for (OpponentRobot robot : allOpponentRobots) {
            if (robot != self && robot.isEnabled()) {
                Pose3d pose = robot.getPose();
                Translation2d min = new Translation2d(
                        pose.getX() - Constants.Frame.FRAME_WITH_BUMPER_LENGTH / 2,
                        pose.getY() - Constants.Frame.FRAME_WITH_BUMPER_WIDTH / 2);
                Translation2d max = new Translation2d(
                        pose.getX() + Constants.Frame.FRAME_WITH_BUMPER_LENGTH / 2,
                        pose.getY() + Constants.Frame.FRAME_WITH_BUMPER_WIDTH / 2);
                boundingBoxes.add(new Pair<>(min, max));
            }
        }
        return boundingBoxes;
    }

    public enum Behavior {
        DEFENSE("defense");

        private final String propertyName;

        private Behavior(String propertyName) {
            this.propertyName = propertyName;
        }

        public String getPropertyName() {
            return propertyName;
        }

        public static Optional<Behavior> fromPropertyName(String propertyName) {
            for (Behavior behavior : values()) {
                if (behavior.getPropertyName().equalsIgnoreCase(propertyName)) {
                    return Optional.of(behavior);
                }
            }
            return Optional.empty();
        }
    }

    private static final Pose2d[] ALLIANCE_START_POSITIONS = new Pose2d[] {
        Constants.FieldStartingLocation.TrenchDepot.getPose(Alliance.Red),
        Constants.FieldStartingLocation.Center.getPose(Alliance.Red),
        Constants.FieldStartingLocation.TrenchOutpost.getPose(Alliance.Red),
        Constants.FieldStartingLocation.TrenchDepot.getPose(Alliance.Blue),
        Constants.FieldStartingLocation.Center.getPose(Alliance.Blue),
        Constants.FieldStartingLocation.TrenchOutpost.getPose(Alliance.Blue)
    };

    private static Pose2d nextStartingPose = new Pose2d(-30, -30, Rotation2d.kZero);
    private static final Lock nextStartingPoseLock = new ReentrantLock(true);

    // Each robot has its own battery source
    private SimulatedBattery batterySource = SimulatedBatteryFactory.create();

    // Create and configure a drivetrain simulation configuration
    private final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default(
                    batterySource)
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
                    COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof, // Use the COF for Neoprene Tread
                    batterySource))
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(
                    Meters.of(Constants.Frame.ROBOT_WHEEL_DISTANCE_LENGTH),
                    Meters.of(Constants.Frame.ROBOT_WHEEL_DISTANCE_WIDTH))
            // Configures the bumper size (dimensions of the robot bumper)
            .withBumperSize(
                    Meters.of(Constants.Frame.FRAME_WITH_BUMPER_LENGTH),
                    Meters.of(Constants.Frame.FRAME_WITH_BUMPER_WIDTH))
            .withRobotMass(Kilograms.of(Constants.Frame.ROBOT_MASS_KG));

    // Create the constraints to use while pathfinding
    private final PathConstraints constraints =
            new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    private final SelfControlledSwerveDriveSimulation driveSimulation;
    private final Pose2d startingPose;

    private final RobotBumpSim robotBumpSim = new RobotBumpSim(new Translation2d[] {
        Constants.Swerve.FRONT_LEFT_WHEEL_LOCATION,
        Constants.Swerve.FRONT_RIGHT_WHEEL_LOCATION,
        Constants.Swerve.BACK_LEFT_WHEEL_LOCATION,
        Constants.Swerve.BACK_RIGHT_WHEEL_LOCATION
    });

    private final Pathfinder pathfinder;
    private final CustomPathfindingCommand pathfindingCommand;

    private ChassisSpeeds pathfindingChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    private boolean enabled = false;
    private int allianceStation = 1;
    private int robotId = 0;
    private Behavior behavior = Behavior.DEFENSE;

    private PIDController xPid = new PIDController(4.0, 0.0, 0.15);
    private PIDController yPid = new PIDController(4.0, 0.0, 0.15);
    private PIDController rPid = new PIDController(4.0, 0.0, 0.3);
    private PIDController rPidGridOrient = new PIDController(4.0, 0.0, 0.3);

    private Translation2d lastTargetPosition = Translation2d.kZero;

    private Pose3d lastSimPose3d = new Pose3d();

    private Timer pinTimer = new Timer();
    private Timer repelTimer = new Timer();

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

        OpponentRobot robot = new OpponentRobot(startingPose, allOpponentRobots.size());
        allOpponentRobots.add(robot);
        return robot;
    }

    private OpponentRobot(Pose2d startingPose, int id) {
        if (Constants.RobotState.getMode() != Mode.SIM) {
            throw new IllegalStateException("OpponentRobot should only be instantiated in simulation mode!");
        }

        this.startingPose = startingPose;
        this.robotId = id;

        rPid.enableContinuousInput(-Math.PI / 4, Math.PI / 4);
        rPidGridOrient.enableContinuousInput(-Math.PI / 4, Math.PI / 4);

        driveSimulation = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(driveTrainSimulationConfig, startingPose));

        pathfinder = new LocalADStar();

        pathfindingCommand = new CustomPathfindingCommand(
                Pose2d.kZero,
                constraints,
                3.0,
                driveSimulation::getActualPoseInSimulationWorld,
                driveSimulation::getActualSpeedsRobotRelative,
                (c, f) -> pathfindingChassisSpeeds = c,
                new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.15), new PIDConstants(4.0, 0.0, 0.3)),
                Constants.Swerve.PP_DEFAULT_CONFIG,
                pathfinder);

        pinTimer.start();
        repelTimer.start();

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
    }

    public Pose3d getPose() {
        return lastSimPose3d;
    }

    public void resetAndEnable() {
        driveSimulation.setSimulationWorldPose(ALLIANCE_START_POSITIONS[allianceStation - 1]);
        lastSimPose3d = new Pose3d(driveSimulation.getActualPoseInSimulationWorld());
        batterySource.setVoltage(SimulatedBatteryFactory.ACTUAL_RESTING_BATTERY_VOLTAGE);
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

    @Override
    public void periodicManaged() {
        if (enabled && RobotState.isEnabled()) {
            switch (behavior) {
                case DEFENSE:
                    runDefense(RobotContainer.model.getRobot());
                    break;
            }
        } else {
            driveSimulation.runChassisSpeeds(new ChassisSpeeds(0, 0, 0), Translation2d.kZero, false, true);
        }

        Pose2d simPose = driveSimulation.getActualPoseInSimulationWorld();

        ChassisSpeeds fieldRelativeSpeeds = driveSimulation.getActualSpeedsFieldRelative();
        lastSimPose3d =
                robotBumpSim.update(simPose, fieldRelativeSpeeds, SimulatedArena.getSimulationSubTicksIn1Period());
        if (robotBumpSim.isOnRamp()) {
            driveSimulation.setSimulationWorldPose(robotBumpSim.getSimWorldPose(simPose));
        }
    }

    private void runDefense(Pose3d targetPose) {

        Pose2d current = driveSimulation.getActualPoseInSimulationWorld();
        Pose2d target = targetPose.toPose2d();

        pathfinder.setDynamicObstacles(getAllOpponentRobotBoundingBoxes(this), current.getTranslation());

        if (lastTargetPosition.getDistance(target.getTranslation()) > 1.0) {
            lastTargetPosition = target.getTranslation();

            pathfindingCommand.targetPose = target;
            pathfindingCommand.initialize();
        }

        pathfindingCommand.execute();

        ChassisSpeeds pidSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xPid.calculate(current.getX(), target.getX()),
                yPid.calculate(current.getY(), target.getY()),
                rPid.calculate(
                        current.getRotation().getRadians(), target.getRotation().getRadians()),
                current.getRotation());

        double targetDistance = target.getTranslation().getDistance(current.getTranslation());

        boolean isRepelling = !repelTimer.hasElapsed(2.0);
        if (isRepelling) {
            pidSpeeds = new ChassisSpeeds(
                    -10.0 / pidSpeeds.vxMetersPerSecond,
                    -10.0 / pidSpeeds.vyMetersPerSecond,
                    pidSpeeds.omegaRadiansPerSecond);
        } else if (targetDistance > 1.35) {
            pinTimer.restart();
        } else if (pinTimer.hasElapsed(4.5)) {
            repelTimer.restart();
        }

        ChassisSpeeds speeds;
        if (targetDistance < 1.5
                || (isRepelling && targetDistance < 3.0 /* at this point just start moving towards them */)) {
            speeds = pidSpeeds;
        } else {
            speeds = pathfindingChassisSpeeds;

            // little boost ;)
            speeds.vxMetersPerSecond *= 1.5;
            speeds.vyMetersPerSecond *= 1.5;

            // grid orient when pathfinding to avoid issues with trench
            speeds.omegaRadiansPerSecond =
                    rPidGridOrient.calculate(current.getRotation().getRadians(), 0);
        }

        driveSimulation.runChassisSpeeds(speeds, Translation2d.kZero, false, true);
    }
}
