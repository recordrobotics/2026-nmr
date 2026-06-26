package frc.robot;

import com.google.common.primitives.ImmutableIntArray;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
// WPILib imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldStartingLocation;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.commands.ShootTuning;
// Local imports
import frc.robot.commands.auto.PlannedAuto;
import frc.robot.control.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.io.real.ClimberReal;
import frc.robot.subsystems.io.real.FeederReal;
import frc.robot.subsystems.io.real.IndexerReal;
import frc.robot.subsystems.io.real.IntakeReal;
import frc.robot.subsystems.io.real.ShooterReal;
import frc.robot.subsystems.io.real.TurretReal;
import frc.robot.subsystems.io.sim.ClimberSim;
import frc.robot.subsystems.io.sim.FeederSim;
import frc.robot.subsystems.io.sim.IndexerSim;
import frc.robot.subsystems.io.sim.IntakeSim;
import frc.robot.subsystems.io.sim.ShooterSim;
import frc.robot.subsystems.io.sim.TurretSim;
import frc.robot.subsystems.shootorchestrator.ShootOrchestrator;
import frc.robot.utils.AutoPath;
import frc.robot.utils.CommandUtils;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.DriverStationUtils.MatchTimeData;
import frc.robot.utils.ExtendedStatusSignalCollection;
import frc.robot.utils.ModuleConstants.InvalidConfigException;
import frc.robot.utils.PositionedSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.RuckigWarmup;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.TalonFXOrchestra;
import frc.robot.utils.field.FieldUtils;
import frc.robot.utils.libraries.Elastic;
import frc.robot.utils.libraries.Elastic.Notification;
import frc.robot.utils.libraries.Elastic.NotificationLevel;
import frc.robot.utils.wrappers.SafeAlert;
import java.util.EnumSet;
import java.util.Objects;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.photonvision.simulation.VisionSystemSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings({"java:S1444", "java:S1104", "java:S3010"})
public final class RobotContainer {

    public static final double ROBOT_PERIODIC = 0.02;
    public static final int CONTROL_JOYSTICK_PORT = 2;
    public static final int CONTROL_XBOX_PORT = 0;

    // Min time remaining in which we can auto reset encoders if not already reset during autonomous
    // (20 - 18 = 2 seconds at the start of auto)
    public static final double FMS_AUTO_RESET_ENCODERS_MIN_TIME = 18;

    public static final ImmutableIntArray NON_HUB_TAG_IDS =
            ImmutableIntArray.of(1, 12, 13, 14, 15, 16, 7, 6, 17, 28, 29, 30, 31, 32, 23, 22);
    public static final double PHOTON_SIM_NOISY_STDDEV_POS = 0.2;
    public static final double PHOTON_SIM_NOISY_STDDEV_ROT = 0.1;

    public enum ShootMode {
        DISABLED,
        AUTO,
        FORCE,
        FIXED
    }

    public static final ExtendedStatusSignalCollection allStatusSignalsToRefresh = new ExtendedStatusSignalCollection();

    public static Drivetrain drivetrain;
    public static PoseSensorFusion poseSensorFusion;
    public static PowerDistributionPanel pdp;
    public static Intake intake;
    public static Turret turret;
    public static Shooter shooter;
    public static Indexer indexer;
    public static Feeder feeder;
    public static Climber climber;
    public static RobotModel model;
    public static FieldStateTracker fieldStateTracker;
    public static ShootOrchestrator shootOrchestrator;
    public static VisionSystemSim visionSim;

    public static TalonFXOrchestra orchestra;

    private static final double ACTUAL_RESTING_BATTERY_VOLTAGE = 12.68;

    private static final double HUB_SCORE_REGISTER_TIME =
            1.25; // takes 1.25 seconds from fuel crossing top of poly to scored
    private static final double HUB_SCORE_TIME =
            1; // hub allows 1 second for fuel to score after deactivation (maybe 3 but 1 safer)

    private static SafeAlert noEncoderResetAlert;
    private static SafeAlert shootModeAlert;

    private static final LoggedDashboardChooser<ShootMode> shootModeChooser = new LoggedDashboardChooser<>("ShootMode");
    private static final LoggedDashboardChooser<AbstractControl> driveMode = new LoggedDashboardChooser<>("Drive Mode");
    private static final LoggedDashboardChooser<FieldStartingLocation> fieldStartingLocationChooser =
            new LoggedDashboardChooser<>("Starting Location");
    private static final LoggedNetworkBoolean resetLocationButton =
            new LoggedNetworkBoolean("Autonomous/ResetLocationButton", false);
    private static final LoggedNetworkBoolean encoderResetButton =
            new LoggedNetworkBoolean("Autonomous/EncoderReset", false);
    private static final LoggedNetworkBoolean recordMatchReplayButton =
            new LoggedNetworkBoolean("RecordMatchReplay", false);
    private static final LoggedNetworkBoolean shootTuningButton = new LoggedNetworkBoolean("ShootTuning", false);

    private static AbstractControl defaultControl;
    private static AbstractControl testControl;

    private static double lastDrivetrainNoReconnectStickyFaultTime = 0;

    private RobotContainer() {
        initialize();
    }

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    private static void initialize() {

        RobotController.setBrownoutVoltage(5.75);

        noEncoderResetAlert = new SafeAlert("Encoders not reset!", AlertType.kError);
        shootModeAlert = new SafeAlert("shoot mode alert", AlertType.kWarning);

        EnumSet.allOf(ShootMode.class).forEach(v -> shootModeChooser.addOption(v.name(), v));
        shootModeChooser.addDefaultOption(ShootMode.AUTO.name(), ShootMode.AUTO);

        EnumSet.allOf(FieldStartingLocation.class)
                .forEach(v -> fieldStartingLocationChooser.addOption(v.toString(), v));
        fieldStartingLocationChooser.addDefaultOption(
                FieldStartingLocation.TrenchDepot.toString(), FieldStartingLocation.TrenchDepot);

        orchestra = new TalonFXOrchestra();

        pdp = new PowerDistributionPanel();

        try {
            drivetrain = new Drivetrain();
        } catch (InvalidConfigException e) {
            ConsoleLogger.logError("Could not load drivetrain config!", e);
            return;
        }

        if (Constants.RobotState.getMode() == Mode.REAL) {
            intake = new Intake(new IntakeReal());
            turret = new Turret(new TurretReal());
            shooter = new Shooter(new ShooterReal());
            indexer = new Indexer(new IndexerReal());
            feeder = new Feeder(new FeederReal());
            climber = new Climber(new ClimberReal());
        } else {
            if (Constants.Vision.VISION_SIMULATION_MODE.isPhotonSim()) {
                visionSim = new VisionSystemSim("main");

                if (Constants.Vision.VISION_SIMULATION_MODE
                        == Constants.Vision.VisionSimulationMode.PHOTON_SIM_INACCURATE) {
                    // simulate misalignment of field elements, assume tags are perfectly placed on field elements
                    AprilTagFieldLayout noisyTagLayout = SimpleMath.addNoiseToAprilTagFieldLayout(
                            Constants.Game.APRILTAG_LAYOUT,
                            NON_HUB_TAG_IDS,
                            PHOTON_SIM_NOISY_STDDEV_POS,
                            PHOTON_SIM_NOISY_STDDEV_POS,
                            0,
                            0,
                            0,
                            PHOTON_SIM_NOISY_STDDEV_ROT);
                    Logger.recordOutput(
                            "PhotonSimTagLayout/Poses",
                            noisyTagLayout.getTags().stream().map(t -> t.pose).toArray(Pose3d[]::new));
                    Logger.recordOutput(
                            "PhotonSimTagLayout/IDs",
                            noisyTagLayout.getTags().stream()
                                    .mapToInt(t -> t.ID)
                                    .toArray());

                    visionSim.addAprilTags(noisyTagLayout);
                } else {
                    visionSim.addAprilTags(Constants.Game.APRILTAG_LAYOUT);
                }
            }

            intake = new Intake(new IntakeSim(ROBOT_PERIODIC, drivetrain.getSwerveDriveSimulation()));
            turret = new Turret(new TurretSim(ROBOT_PERIODIC));
            shooter = new Shooter(new ShooterSim(ROBOT_PERIODIC));
            indexer = new Indexer(new IndexerSim(ROBOT_PERIODIC));
            feeder = new Feeder(new FeederSim(ROBOT_PERIODIC));
            climber = new Climber(new ClimberSim(ROBOT_PERIODIC, drivetrain.getSwerveDriveSimulation()));
        }

        poseSensorFusion = new PoseSensorFusion(getStartingLocation().getPose());
        fieldStateTracker = new FieldStateTracker();
        shootOrchestrator = new ShootOrchestrator();

        model = new RobotModel();

        // Sets up auto path
        AutoPath.initialize();
        // Warmup Ruckig
        CommandScheduler.getInstance().schedule(RuckigWarmup.warmupCommand());

        AutoPath.setupAutoChooser();
        PlannedAuto.setAutoSupplier(AutoPath::getAutoChooser);

        // Sets up Control scheme chooser
        addControls(
                new XboxControls(CONTROL_XBOX_PORT),
                new JoystickControls(CONTROL_JOYSTICK_PORT),
                new SwitchControls(CONTROL_XBOX_PORT));

        configureTriggers();

        noEncoderResetAlert.set(true);

        if (Constants.RobotState.getMode() != Mode.REAL) {
            // No point in manually resetting encoders in simulation since starting config is always in the right spot
            resetEncoders();
            CommandScheduler.getInstance()
                    .schedule(Commands.waitSeconds(0.04)
                            .andThen(Commands.runOnce(RobotContainer::resetEncoders))
                            .ignoringDisable(true));

            // Register all powered subsystems with the simulation battery
            registerPoweredSubsystems(intake, turret, shooter, indexer, feeder, climber);
            SimulatedBattery.setVoltage(ACTUAL_RESTING_BATTERY_VOLTAGE);
            SimulatedBattery.setDischargeRate(0.02 / 378.45);
        }
    }

    public static void teleopInit() {
        DriverStationUtils.teleopInit();
    }

    public static void recordMatchReplay() {
        poseSensorFusion.recordMatchReplay();
        fieldStateTracker.recordMatchReplay();

        Elastic.sendNotification(new Notification(
                NotificationLevel.INFO,
                "Match Replay Recorded!",
                "The last 2:45 minutes have been recorded on all supported cameras (LL4).",
                10000));
    }

    public static void disabledInit() {
        // trigger recording of match replay if time left is <= 1 seconds and not autonomous
        if (DriverStation.isFMSAttached() && DriverStation.getMatchTime() <= 1 && !DriverStation.isAutonomous()) {
            recordMatchReplay();
        }
    }

    public static void autonomousInit() {
        // FMS only reset encoders when enabling autonomous for the first time if not already reset (safety measure)
        // Time limit is so that if the robot restarts during the auto we don't want to reset encoders
        // Don't want to autoreset in teleop because motors keep their positions if you don't power cycle
        if (DriverStation.isFMSAttached()
                && DriverStation.getMatchTime() >= FMS_AUTO_RESET_ENCODERS_MIN_TIME
                && noEncoderResetAlert.get()) {
            resetEncoders();
        }
    }

    public static void disabledExit() {
        // For the generic any enable only reset if not connected to FMS
        // because we don't want to reset encoders if robot restarts in the middle of the match
        // since the motors keep their positions if you don't power cycle
        // for FMS handling see autonomousInit
        // Reset encoders when enabling if not already reset (safety measure)
        if (!DriverStation.isFMSAttached() && noEncoderResetAlert.get()) {
            resetEncoders();
        }
    }

    private static boolean shouldBeShooting() {

        MatchTimeData matchData = DriverStationUtils.getMatchTimeData();

        SmartDashboard.putNumber("ShiftTime", Math.ceil(matchData.timeLeftInShift()));
        SmartDashboard.putString("CurrentShift", matchData.shift().getName());
        SmartDashboard.putBoolean("WonAuto", matchData.wonAuto());

        Logger.recordOutput("HubActive", matchData.currentHubActive());

        if (shootModeChooser.get() == null) return false;

        switch (shootModeChooser.get()) {
            case FORCE:
                return true;
            case DISABLED:
                return false;
            case AUTO:
                double timeToScore = shootOrchestrator.getShotTimeOfFlight() + HUB_SCORE_REGISTER_TIME;
                boolean shouldAutoShoot = matchData.currentHubActive()
                        // start early if next shift active
                        || (matchData.nextHubActive() && matchData.timeLeftInShift() <= timeToScore)
                        ||
                        // shoot even after shift deactivates while fuel is still being scored
                        (matchData.previousHubActive() && matchData.timeSinceShift() <= HUB_SCORE_TIME - timeToScore);

                if (FieldUtils.isInAllianceZone() && shouldAutoShoot) {
                    return !getControl().isShooterDisableShootPressed();
                } else {
                    return getControl().isShooterPassPressed();
                }
            case FIXED:
                return getControl().isShooterPassPressed();
            default:
                return false;
        }
    }

    private static void returnToOverviewTabIfIntakeStarting() {
        if (intake.getTargetState() == IntakeState.STARTING && DriverStation.isTeleopEnabled()) {
            Elastic.selectTab("Overview");
        }
    }

    private static void configureTriggers() {
        new Trigger(() -> getControl().isIntakeInvertPressed())
                .onTrue(Commands.runOnce(
                        () -> {
                            returnToOverviewTabIfIntakeStarting();
                            intake.setState(Intake.IntakeState.RETRACTED);
                            climber.setState(Constants.ClimberHeight.DOWN);
                        },
                        intake,
                        climber))
                .onFalse(Commands.runOnce(
                        () -> {
                            returnToOverviewTabIfIntakeStarting();
                            intake.setState(Intake.IntakeState.INTAKE);
                            climber.setState(Constants.ClimberHeight.DOWN);
                        },
                        intake,
                        climber));

        new Trigger(() -> getControl().isIntakePressed())
                .onTrue(Commands.runOnce(
                        () -> {
                            returnToOverviewTabIfIntakeStarting();
                            intake.setState(Intake.IntakeState.INTAKE);
                            climber.setState(Constants.ClimberHeight.DOWN);
                        },
                        intake,
                        climber))
                .onFalse(Commands.runOnce(
                        () -> {
                            returnToOverviewTabIfIntakeStarting();
                            intake.setState(Intake.IntakeState.OUT);
                            climber.setState(Constants.ClimberHeight.DOWN);
                        },
                        intake,
                        climber));

        new Trigger(() -> getControl().isIntakeUpPressed())
                .onTrue(Commands.runOnce(
                        () -> {
                            returnToOverviewTabIfIntakeStarting();
                            intake.setState(Intake.IntakeState.RETRACTED);
                            climber.setState(Constants.ClimberHeight.DOWN);
                        },
                        intake,
                        climber))
                .onFalse(Commands.runOnce(
                        () -> {
                            returnToOverviewTabIfIntakeStarting();
                            intake.setState(Intake.IntakeState.OUT);
                            climber.setState(Constants.ClimberHeight.DOWN);
                        },
                        intake,
                        climber));

        new Trigger(() -> getControl().isReverseIntakePressed())
                .onTrue(Commands.runOnce(
                        () -> {
                            returnToOverviewTabIfIntakeStarting();
                            intake.setState(Intake.IntakeState.EJECT);
                            climber.setState(Constants.ClimberHeight.DOWN);
                        },
                        intake,
                        climber))
                .onFalse(Commands.runOnce(
                        () -> {
                            returnToOverviewTabIfIntakeStarting();
                            intake.setState(Intake.IntakeState.OUT);
                            climber.setState(Constants.ClimberHeight.DOWN);
                        },
                        intake,
                        climber));

        new Trigger(() -> getControl().isDefenseModePressed())
                .onTrue(Commands.runOnce(
                        () -> {
                            intake.setState(Intake.IntakeState.STARTING);
                            Elastic.selectTab("Defense");
                        },
                        intake));

        new Trigger(() -> getControl().isClimbPressed())
                .onTrue(Commands.runOnce(
                        () -> {
                            if (climber.getNearestHeight() == Constants.ClimberHeight.DOWN) {
                                climber.setState(Constants.ClimberHeight.UP);
                                intake.setState(Intake.IntakeState.STARTING);
                                Elastic.selectTab("Defense");
                            } else {
                                climber.setState(Constants.ClimberHeight.DOWN);
                                Elastic.selectTab("Overview");
                            }
                        },
                        climber,
                        intake));

        new Trigger(RobotContainer::shouldBeShooting)
                .onTrue(Commands.runOnce(
                                () -> {
                                    shootOrchestrator.setEnableShooting(true);
                                    shootOrchestrator.setFixedMode(shootModeChooser.get() == ShootMode.FIXED);
                                },
                                shooter)
                        .ignoringDisable(true))
                .onFalse(Commands.runOnce(
                                () -> {
                                    shootOrchestrator.setEnableShooting(false);
                                    shootOrchestrator.setFixedMode(shootModeChooser.get() == ShootMode.FIXED);
                                },
                                shooter)
                        .ignoringDisable(true));

        // Kill subsystems trigger
        /*
         * All subsystems automatically re-enable after 2 periodic cycles unless holding kill trigger to prevent accidental disables
         * When button is released all commands are canceled
         */
        new Trigger(() -> getControl().isKillTriggered())
                .whileTrue(CommandUtils.setForceDisabledForAllCommand(
                                true, 2, intake, shooter, climber, feeder, indexer, turret)
                        .repeatedly())
                .onFalse(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

        // Reset pose trigger
        new Trigger(() -> getControl().isPoseResetTriggered())
                .onTrue(Commands.either(
                                Commands.runOnce(() -> poseSensorFusion.setToPose(
                                                getStartingLocation().getPose()))
                                        .ignoringDisable(true),
                                Commands.runOnce(poseSensorFusion::alignRotationWithDriverStation),
                                () -> Constants.RobotState.getMode() == Mode.SIM)
                        .ignoringDisable(Constants.RobotState.getMode() == Mode.SIM));
        new Trigger(resetLocationButton)
                .onTrue(Commands.runOnce(() -> {
                            resetLocationButton.set(false);
                            poseSensorFusion.setToPose(getStartingLocation().getPose());
                            if (Constants.RobotState.getMode() == Mode.SIM) {
                                // reset voltage
                                SimulatedBattery.setVoltage(ACTUAL_RESTING_BATTERY_VOLTAGE);
                            }
                        })
                        .ignoringDisable(true));
        new Trigger(encoderResetButton)
                .onTrue(Commands.runOnce(() -> {
                            encoderResetButton.set(false);
                            resetEncoders();
                        })
                        .ignoringDisable(true));

        new Trigger(recordMatchReplayButton)
                .onTrue(Commands.runOnce(() -> {
                            recordMatchReplayButton.set(false);
                            recordMatchReplay();
                        })
                        .ignoringDisable(true));

        new Trigger(shootTuningButton)
                .onTrue(Commands.runOnce(() -> shootTuningButton.set(false)).andThen(new ShootTuning()));
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public static Command getAutonomousCommand() {
        if (SysIdManager.getProvider().isEnabled()) {
            return SysIdManager.getProvider().createCommand();
        }

        return PlannedAuto.getAutoCommand();
    }

    public static void robotPeriodic() {
        allStatusSignalsToRefresh.refreshAll();
        PositionedSubsystem.PositionedSubsystemManager.getInstance().update();

        ShootMode currentShootMode = shootModeChooser.get();
        if (currentShootMode == null) {
            shootModeAlert.setText("Shoot mode is <UNKNOWN>");
            shootModeAlert.set(true);
        } else if (currentShootMode != ShootMode.AUTO) {
            shootModeAlert.setText("Shoot mode is " + currentShootMode.name());
            shootModeAlert.set(true);
        } else {
            shootModeAlert.set(false);
        }

        if (drivetrain.hasRegainedDrivetrainConnectionStickyFault()) {
            AbstractControl control = getControl();
            // vibrate for first 3 seconds or while no user input
            if (Timer.getTimestamp() - lastDrivetrainNoReconnectStickyFaultTime < 3 || !control.hasUserInput()) {
                control.vibrate(RumbleType.kBothRumble, 1.0);
            } else {
                // clear sticky fault after 3 seconds and once user starts giving input
                control.vibrate(RumbleType.kBothRumble, 0.0);
                drivetrain.clearRegainedDrivetrainConnectionStickyFault();
            }
        } else {
            lastDrivetrainNoReconnectStickyFaultTime = Timer.getTimestamp();
        }
    }

    public static void simulationPeriodic() {
        if (Constants.Vision.VISION_SIMULATION_MODE.isPhotonSim()) {
            visionSim.update(model.getRobot());
        }
    }

    public static void registerPoweredSubsystems(PoweredSubsystem... subsystems) {
        for (PoweredSubsystem subsystem : subsystems) {
            SimulatedBattery.addElectricalAppliances(subsystem::getCurrentDraw);
        }
    }

    public static void resetEncoders() {
        PositionedSubsystem.PositionedSubsystemManager.getInstance().resetAll();
        model.intakeModel.resetHopperExtension();

        noEncoderResetAlert.set(false);
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Encoders reset!", "Successfully reset encoders."));
    }

    /**
     * Initializes the control object
     *
     * @param defaultControl the first term will always be the default control object
     * @param controls any other control objects you want to initialize
     */
    public static void addControls(AbstractControl defaultControl, AbstractControl... controls) {
        RobotContainer.defaultControl = defaultControl;

        // Sets up drive mode options
        driveMode.addDefaultOption(defaultControl.toDisplayName(), defaultControl);
        for (AbstractControl abstractControl : controls) {
            driveMode.addOption(abstractControl.toDisplayName(), abstractControl);
        }
    }

    public static void setTestControl(AbstractControl testControl) {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.TEST) return;
        RobotContainer.testControl = testControl;
    }

    public static AbstractControl getControl() {
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.TEST) {
            if (testControl == null) {
                throw new IllegalStateException("Test control is not set!");
            }
            return testControl;
        }

        if (driveMode.get() == null) return defaultControl;
        return driveMode.get();
    }

    public static FieldStartingLocation getStartingLocation() {
        return Objects.requireNonNullElse(fieldStartingLocationChooser.get(), FieldStartingLocation.TrenchDepot);
    }

    /** frees up all hardware allocations */
    public static void close() {
        drivetrain.close();
        poseSensorFusion.close();
        intake.close();
        shooter.close();
        climber.close();
        turret.close();
        indexer.close();
        feeder.close();
    }
}
