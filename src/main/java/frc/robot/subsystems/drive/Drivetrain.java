package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.subsystems.io.real.SwerveModuleReal;
import frc.robot.subsystems.io.sim.SwerveModuleSim;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ModuleConstants.InvalidConfigException;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import frc.robot.utils.TalonFXOrchestra;
import frc.robot.utils.libraries.Elastic;
import frc.robot.utils.libraries.Elastic.Notification;
import frc.robot.utils.libraries.Elastic.NotificationLevel;
import frc.robot.utils.libraries.bumpsim.RobotBumpSim;
import frc.robot.utils.modifiers.ControlModifierService;
import frc.robot.utils.modifiers.ControlModifierService.ControlModifier;
import frc.robot.utils.modifiers.DrivetrainControl;
import frc.robot.utils.wrappers.SafeAlert;
import java.util.Arrays;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;

/** Represents a swerve drive style drivetrain. */
public final class Drivetrain extends ManagedSubsystemBase {

    private static final int FL = 0;
    private static final int FR = 1;
    private static final int BL = 2;
    private static final int BR = 3;

    private static final SwerveModuleState[] X_HOLD_STATE = {
        new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(315.0)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(135.0)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(225.0))
    };

    private record PDPChannel(int driveChannel, int turnChannel) {}

    private static final PDPChannel[] MODULE_PDP_CHANNELS = {
        new PDPChannel(0, 1), new PDPChannel(2, 3), new PDPChannel(6, 7), new PDPChannel(4, 5)
    };
    private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};
    private static final int[] MODULE_DRIVE_TRACKS = {
        TalonFXOrchestra.Tracks.FL_DRIVE,
        TalonFXOrchestra.Tracks.FR_DRIVE,
        TalonFXOrchestra.Tracks.BL_DRIVE,
        TalonFXOrchestra.Tracks.BR_DRIVE
    };
    private static final int[] MODULE_TURN_TRACKS = {
        TalonFXOrchestra.Tracks.FL_TURN,
        TalonFXOrchestra.Tracks.FR_TURN,
        TalonFXOrchestra.Tracks.BL_TURN,
        TalonFXOrchestra.Tracks.BR_TURN
    };

    private static final boolean DEBUG_LOG_MODIFIERS = false;

    private static final Velocity<VoltageUnit> SYSID_DRIVE_RAMP_RATE =
            Volts.of(3.0).per(Second);
    private static final Voltage SYSID_DRIVE_STEP_VOLTAGE = Volts.of(3.0);
    private static final Time SYSID_DRIVE_TIMEOUT = Seconds.of(1.5);

    private static final Velocity<VoltageUnit> SYSID_TURN_RAMP_RATE =
            Volts.of(6.0).per(Second);
    private static final Voltage SYSID_TURN_STEP_VOLTAGE = Volts.of(2.0);
    private static final Time SYSID_TURN_TIMEOUT = Seconds.of(1.0);

    // Creates swerve module objects
    private final SwerveModule[] modules;

    private final SysIdRoutine sysIdRoutineDriveMotorsSpin;
    private final SysIdRoutine sysIdRoutineDriveMotorsForward;
    private final SysIdRoutine sysIdRoutineTurnMotors;

    // Creates swerve kinematics
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            Constants.Swerve.FRONT_LEFT_WHEEL_LOCATION,
            Constants.Swerve.FRONT_RIGHT_WHEEL_LOCATION,
            Constants.Swerve.BACK_LEFT_WHEEL_LOCATION,
            Constants.Swerve.BACK_RIGHT_WHEEL_LOCATION);

    // Create and configure a drivetrain simulation configuration
    private final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
            // Specify gyro type (for realistic gyro drifting and error simulation)
            // .withGyro(() -> new GyroSimulation(0.5, 0.05)) // navX-Micro
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

    private final SwerveDriveSimulation swerveDriveSimulation;
    private final RobotBumpSim robotBumpSim;

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    private int lastModifiersAppliedCount = 0;
    private SwerveModuleState[] lastModuleSetpoints = new SwerveModuleState[0];

    private Pose3d lastSimPose3d = new Pose3d();

    private boolean useXHold = false;

    private final SafeAlert lostDrivetrainAlert = new SafeAlert("Drivetrain lost!", AlertType.kError);
    private final Debouncer lostDrivetrainDebouncer = new Debouncer(1.5, Debouncer.DebounceType.kBoth);
    private boolean prevLostDrivetrain = false;
    private boolean regainedDrivetrainStickyFault = false;

    public Drivetrain() throws InvalidConfigException {
        ModuleConstants[] moduleConstants = {
            Constants.Swerve.getFrontLeftConstants(),
            Constants.Swerve.getFrontRightConstants(),
            Constants.Swerve.getBackLeftConstants(),
            Constants.Swerve.getBackRightConstants()
        };

        SwerveModuleIO[] moduleIO = new SwerveModuleIO[moduleConstants.length];

        if (Constants.RobotState.getMode() == Mode.REAL) {
            swerveDriveSimulation = null;
            robotBumpSim = null;

            for (int i = 0; i < moduleConstants.length; i++) {
                moduleIO[i] = new SwerveModuleReal(moduleConstants[i], MODULE_DRIVE_TRACKS[i], MODULE_TURN_TRACKS[i]);
            }
        } else {
            /* Create a swerve drive simulation */
            swerveDriveSimulation = new SwerveDriveSimulation(
                    // Specify Configuration
                    driveTrainSimulationConfig,
                    // Specify starting pose
                    RobotContainer.getStartingLocation().getPose());

            // Register the drivetrain simulation to the default simulation world
            SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);

            for (int i = 0; i < moduleConstants.length; i++) {
                moduleIO[i] = new SwerveModuleSim(
                        swerveDriveSimulation.getModules()[i],
                        moduleConstants[i],
                        MODULE_DRIVE_TRACKS[i],
                        MODULE_TURN_TRACKS[i],
                        MODULE_PDP_CHANNELS[i].driveChannel(),
                        MODULE_PDP_CHANNELS[i].turnChannel());
            }

            robotBumpSim = new RobotBumpSim(kinematics.getModules());
        }

        modules = new SwerveModule[moduleConstants.length];
        for (int i = 0; i < moduleConstants.length; i++) {
            modules[i] = new SwerveModule(MODULE_NAMES[i], moduleConstants[i], moduleIO[i]);
        }

        setpointGenerator = new SwerveSetpointGenerator(
                Constants.Swerve.PP_DEFAULT_CONFIG,
                Units.rotationsToRadians(
                        Constants.Swerve.TURN_MAX_ANGULAR_VELOCITY) // The max rotation velocity of a swerve module in
                // radians per second
                );

        // Initialize the previous setpoint to the robot's current speeds & module states
        ChassisSpeeds currentSpeeds = getChassisSpeeds();
        SwerveModuleState[] currentStates = getModuleStates();
        previousSetpoint = new SwerveSetpoint(
                currentSpeeds, currentStates, DriveFeedforwards.zeros(Constants.Swerve.PP_DEFAULT_CONFIG.numModules));

        sysIdRoutineDriveMotorsSpin = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_DRIVE_RAMP_RATE,
                        SYSID_DRIVE_STEP_VOLTAGE,
                        SYSID_DRIVE_TIMEOUT,
                        state -> Logger.recordOutput("Drivetrain/Drive/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(this::sysIdOnlyDriveMotorsSpin, null, this));

        sysIdRoutineDriveMotorsForward = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_DRIVE_RAMP_RATE,
                        SYSID_DRIVE_STEP_VOLTAGE,
                        SYSID_DRIVE_TIMEOUT,
                        state -> Logger.recordOutput("Drivetrain/Drive/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(this::sysIdOnlyDriveMotorsForward, null, this));

        sysIdRoutineTurnMotors = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_TURN_RAMP_RATE,
                        SYSID_TURN_STEP_VOLTAGE,
                        SYSID_TURN_TIMEOUT,
                        state -> Logger.recordOutput("Drivetrain/Turn/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(this::sysIdOnlyTurnMotors, null, this));
    }

    public SwerveDriveSimulation getSwerveDriveSimulation() {
        return swerveDriveSimulation;
    }

    private static DrivetrainControl getDrivetrainControl() {
        if (RobotState.isTeleop()) {
            return RobotContainer.getControl().getDrivetrainControl();
        } else {
            return DrivetrainControl.createRobotRelative(Transform2d.kZero, Transform2d.kZero, Transform2d.kZero);
        }
    }

    private static double projectFeedforward(double robotForceX, double robotForceY, double wheelAngleRadians) {
        double robotAccelX = robotForceX / Constants.Frame.ROBOT_MASS_KG;
        double robotAccelY = robotForceY / Constants.Frame.ROBOT_MASS_KG;

        double robotVoltageX = robotAccelX * Constants.Swerve.FEEDFORWARD_KA;
        double robotVoltageY = robotAccelY * Constants.Swerve.FEEDFORWARD_KA;

        return Math.cos(wheelAngleRadians) * robotVoltageX + Math.sin(wheelAngleRadians) * robotVoltageY;
    }

    /** Drives the robot using robot relative ChassisSpeeds. */
    private void driveInternal() {
        DrivetrainControl drivetrainControl = getDrivetrainControl();

        int applyCount = 0;

        for (ControlModifier prioritizedModifier :
                ControlModifierService.getInstance().getModifiers()) {
            if (prioritizedModifier.modifier().isEnabled()
                    && (DEBUG_LOG_MODIFIERS
                            ? prioritizedModifier.modifier().logApply(drivetrainControl)
                            : prioritizedModifier.modifier().apply(drivetrainControl))) {
                applyCount++;
            }
        }

        lastModifiersAppliedCount = applyCount;

        ChassisSpeeds nonDiscreteSpeeds = drivetrainControl.toChassisSpeeds(); // Converts the control to ChassisSpeeds
        double[] robotRelativeForcesXNewtons = drivetrainControl.robotRelativeForcesXNewtons();
        double[] robotRelativeForcesYNewtons = drivetrainControl.robotRelativeForcesYNewtons();

        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint, // The previous setpoint
                nonDiscreteSpeeds, // The desired target speeds
                RobotContainer.ROBOT_PERIODIC // The loop time of the robot code, in seconds
                );

        double[] setpointFFX = previousSetpoint.feedforwards().robotRelativeForcesXNewtons();
        double[] setpointFFY = previousSetpoint.feedforwards().robotRelativeForcesYNewtons();

        SwerveModuleState[] swerveModuleStates = previousSetpoint.moduleStates();

        if (Math.abs(previousSetpoint.robotRelativeSpeeds().vxMetersPerSecond) < 1E-3
                && Math.abs(previousSetpoint.robotRelativeSpeeds().vyMetersPerSecond) < 1E-3
                && Math.abs(previousSetpoint.robotRelativeSpeeds().omegaRadiansPerSecond) < 1E-3
                && useXHold) {
            swerveModuleStates = X_HOLD_STATE;
        }

        if (!(SysIdManager.getProvider() instanceof SysIdSpin)
                && !(SysIdManager.getProvider() instanceof SysIdForward)) {
            SwerveModuleState[] states = RobotContainer.drivetrain.getModuleStates();
            for (int i = 0; i < swerveModuleStates.length; i++) {
                modules[i].setDesiredState(
                        swerveModuleStates[i],
                        projectFeedforward(
                                robotRelativeForcesXNewtons[i] + setpointFFX[i],
                                robotRelativeForcesYNewtons[i] + setpointFFY[i],
                                states[i].angle.getRadians()));
            }
        }

        lastModuleSetpoints = swerveModuleStates;
    }

    /**
     * Checks if all the drive and turn motors are connected
     * (encoders only matter at the start)
     * @return  true if all modules are connected, false if at least one module is not connected
     */
    public boolean isConnected() {
        for (SwerveModule module : modules) {
            if (!module.isConnected()) {
                return false;
            }
        }

        return true;
    }

    @Override
    public void periodicManaged() {
        driveInternal();

        for (SwerveModule module : modules) {
            module.periodic();
        }

        lostDrivetrainAlert.set(!isConnected());
        boolean lost = lostDrivetrainDebouncer.calculate(!isConnected());
        if (lost && !prevLostDrivetrain) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.ERROR,
                    "Drivetrain lost!",
                    "One or more swerve modules have lost connection to motors.",
                    10000));
        } else if (!lost && prevLostDrivetrain) {
            Elastic.sendNotification(new Notification(
                    NotificationLevel.INFO,
                    "Drivetrain connection restored",
                    "All swerve modules have regained connection to motors.",
                    0 /* no dismiss */));
            regainedDrivetrainStickyFault = true;
        }
        prevLostDrivetrain = lost;
    }

    public boolean hasRegainedDrivetrainConnectionStickyFault() {
        return regainedDrivetrainStickyFault;
    }

    public void clearRegainedDrivetrainConnectionStickyFault() {
        regainedDrivetrainStickyFault = false;
    }

    @Override
    public void simulationPeriodicManaged() {
        for (SwerveModule module : modules) {
            module.simulationPeriodic();
        }

        Pose2d simPose = swerveDriveSimulation.getSimulatedDriveTrainPose();

        ChassisSpeeds fieldRelativeSpeeds = swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
        lastSimPose3d =
                robotBumpSim.update(simPose, fieldRelativeSpeeds, SimulatedArena.getSimulationSubTicksIn1Period());
        if (robotBumpSim.isOnRamp()) {
            swerveDriveSimulation.setSimulationWorldPose(robotBumpSim.getSimWorldPose(simPose));
        }
    }

    public void sysIdOnlyDriveMotorsSpin(Voltage volts) {
        modules[FL].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(360 - 45.0)), 0);
        modules[FR].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(180 + 45.0)), 0);
        modules[BL].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)), 0);
        modules[BR].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(90 + 45.0)), 0);

        for (SwerveModule module : modules) {
            module.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
        }
    }

    public void sysIdOnlyDriveMotorsForward(Voltage volts) {
        SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

        for (SwerveModule module : modules) {
            module.setDesiredState(state, 0);
            module.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
        }
    }

    @AutoLogLevel(level = Level.SYSID)
    public double sysIdOnlyGetDriveMotorVolts() {
        return SimpleMath.average(Arrays.stream(modules)
                        .mapToDouble(SwerveModule::getDriveMotorVoltage)
                        .toArray())
                .orElseThrow();
    }

    @AutoLogLevel(level = Level.SYSID)
    public double sysIdOnlyGetDriveMotorPosition() {
        return SimpleMath.average(Arrays.stream(modules)
                        .mapToDouble(s -> s.getModulePosition().distanceMeters)
                        .toArray())
                .orElseThrow();
    }

    @AutoLogLevel(level = Level.SYSID)
    public double sysIdOnlyGetDriveMotorVelocity() {
        return SimpleMath.average(Arrays.stream(modules)
                        .mapToDouble(s -> s.getModuleState().speedMetersPerSecond)
                        .toArray())
                .orElseThrow();
    }

    public void sysIdOnlyTurnMotors(Voltage volts) {
        for (SwerveModule module : modules) {
            module.setTurnMotorVoltsSysIdOnly(volts.in(Volts));
        }
    }

    @AutoLogLevel(level = Level.SYSID)
    public double sysIdOnlyGetTurnMotorVolts() {
        return SimpleMath.average(Arrays.stream(modules)
                        .mapToDouble(SwerveModule::getTurnMotorVoltage)
                        .toArray())
                .orElseThrow();
    }

    @AutoLogLevel(level = Level.SYSID)
    public double sysIdOnlyGetTurnMotorPosition() {
        return SimpleMath.average(Arrays.stream(modules)
                        .mapToDouble(s -> s.getModuleState().angle.getRotations())
                        .toArray())
                .orElseThrow();
    }

    @AutoLogLevel(level = Level.SYSID)
    public double sysIdOnlyGetTurnMotorVelocity() {
        return SimpleMath.average(Arrays.stream(modules)
                        .mapToDouble(SwerveModule::getTurnWheelVelocity)
                        .toArray())
                .orElseThrow();
    }

    /**
     * Retrieves the current chassis speeds relative to the robot's orientation.
     *
     * <p>This method calculates the chassis speeds based on the current states of all four swerve
     * modules using the drivetrain's kinematics.
     *
     * @return The current relative chassis speeds as a ChassisSpeeds object.
     */
    @AutoLogLevel(level = Level.REAL)
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(
                Arrays.stream(modules).map(SwerveModule::getModuleState).toArray(SwerveModuleState[]::new));
    }

    /**
     * Retrieves the current chassis acceleration relative to the robot's orientation.
     *
     * <p>This method calculates the chassis acceleration based on the current states of all four
     * swerve modules using the drivetrain's kinematics.
     *
     * @return The current relative chassis acceleration as a ChassisSpeeds object.
     */
    @AutoLogLevel(level = Level.REAL)
    public ChassisSpeeds getChassisAcceleration() {
        return kinematics.toChassisSpeeds(Arrays.stream(modules)
                .map(SwerveModule::getModuleStateAcceleration)
                .toArray(SwerveModuleState[]::new));
    }

    @AutoLogLevel(level = Level.REAL)
    public int getModifiersAppliedCount() {
        return lastModifiersAppliedCount;
    }

    /**
     * Returns the swerve drive kinematics for this drivetrain.
     *
     * @return The SwerveDriveKinematics object associated with this drivetrain.
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public Pose3d getLastSimPose3d() {
        return lastSimPose3d;
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(SwerveModule::getModulePosition).toArray(SwerveModulePosition[]::new);
    }

    @AutoLogLevel(level = Level.REAL)
    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(modules).map(SwerveModule::getModuleState).toArray(SwerveModuleState[]::new);
    }

    @AutoLogLevel(level = Level.REAL)
    public SwerveModuleState[] getModuleSetpoints() {
        return lastModuleSetpoints;
    }

    public Command sysIdQuasistaticDriveMotorsSpin(SysIdRoutine.Direction direction) {
        return sysIdRoutineDriveMotorsSpin.quasistatic(direction);
    }

    public Command sysIdDynamicDriveMotorsSpin(SysIdRoutine.Direction direction) {
        return sysIdRoutineDriveMotorsSpin.dynamic(direction);
    }

    public Command sysIdQuasistaticDriveMotorsForward(SysIdRoutine.Direction direction) {
        return sysIdRoutineDriveMotorsForward.quasistatic(direction);
    }

    public Command sysIdDynamicDriveMotorsForward(SysIdRoutine.Direction direction) {
        return sysIdRoutineDriveMotorsForward.dynamic(direction);
    }

    public Command sysIdQuasistaticTurnMotors(SysIdRoutine.Direction direction) {
        return sysIdRoutineTurnMotors.quasistatic(direction);
    }

    public Command sysIdDynamicTurnMotors(SysIdRoutine.Direction direction) {
        return sysIdRoutineTurnMotors.dynamic(direction);
    }

    /** frees up all hardware allocations */
    @Override
    public void close() {
        for (SwerveModule module : modules) {
            module.close();
        }
    }

    public static class SysIdTurn implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.drivetrain.sysIdQuasistaticTurnMotors(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.drivetrain.sysIdDynamicTurnMotors(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }

        @Override
        public boolean isReversed() {
            return false;
        }
    }

    public static class SysIdSpin implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.drivetrain.sysIdQuasistaticDriveMotorsSpin(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.drivetrain.sysIdDynamicDriveMotorsSpin(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }

        @Override
        public boolean isReversed() {
            return false;
        }
    }

    public static class SysIdForward implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.drivetrain.sysIdQuasistaticDriveMotorsForward(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.drivetrain.sysIdDynamicDriveMotorsForward(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }

        @Override
        public boolean isReversed() {
            return false;
        }
    }
}
