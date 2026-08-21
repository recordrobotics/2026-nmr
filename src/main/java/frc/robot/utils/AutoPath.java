package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.CreateAutoRoutineException;
import frc.robot.commands.auto.IAutoRoutine;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.utils.libraries.Elastic.NotificationLevel;
import frc.robot.utils.modifiers.AutoControlModifier;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class AutoPath {

    private static LoggedDashboardChooser<Command> autoChooser;

    private static final ProfiledPIDController spinController = new ProfiledPIDController(
            Constants.Control.SPIN_KP, 0, Constants.Control.SPIN_KD, Constants.Control.SPIN_CONSTRAINTS);

    private AutoPath() {}

    /**
     * Sets up the auto chooser with the given routines in addition to pathplanner autos loaded by default.
     * @param routines the auto routines to add to the chooser
     */
    @SafeVarargs
    public static final void setupAutoChooser(final Supplier<IAutoRoutine>... routines) {
        autoChooser = new LoggedDashboardChooser<>("Auto Code", AutoBuilder.buildAutoChooser());

        // Add non-pathplanner autos
        for (Supplier<IAutoRoutine> routineSupplier : routines) {

            IAutoRoutine routine;
            try {
                routine = routineSupplier.get();
            } catch (CreateAutoRoutineException e) {
                ConsoleLogger.logError("Can't get auto routine", e);
                continue;
            }

            if (routine instanceof Command cmd) {
                autoChooser.addOption(routine.getAutoName(), cmd);
            } else {
                throw new IllegalArgumentException("Auto routine does not implement Command: " + routine.getAutoName());
            }
        }
    }

    public static Command getAutoChooser() {
        return autoChooser.get();
    }

    @SuppressWarnings("java:S109")
    public static void initialize() {

        spinController.enableContinuousInput(-Math.PI, Math.PI);
        spinController.setTolerance(Units.degreesToRadians(6));

        RobotConfig config = Constants.Swerve.PP_DEFAULT_CONFIG;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            ConsoleLogger.logError("AutoPath failed to load config", e);
            Notifications.send(NotificationLevel.ERROR, "AutoPath failed to load config", "Error message in console");
        }

        // Registering named commands (so that the pathplanner can call them by name)
        NamedCommands.registerCommand(
                "Intake",
                Commands.run(() -> RobotContainer.intake.setState(IntakeState.INTAKE), RobotContainer.intake));
        NamedCommands.registerCommand(
                "IntakeOff",
                Commands.runOnce(() -> RobotContainer.intake.setState(IntakeState.OUT), RobotContainer.intake));
        NamedCommands.registerCommand(
                "IntakeDepot",
                Commands.run(() -> RobotContainer.intake.setState(IntakeState.INTAKE), RobotContainer.intake)
                        .withTimeout(2.2));
        NamedCommands.registerCommand(
                "Mixer",
                Commands.repeatingSequence(
                        Commands.runOnce(
                                () -> RobotContainer.intake.setState(IntakeState.RETRACTED), RobotContainer.intake),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> RobotContainer.intake.setState(IntakeState.OUT), RobotContainer.intake),
                        Commands.waitSeconds(0.5)));
        NamedCommands.registerCommand(
                "Pass",
                Commands.startEnd(
                        () -> RobotContainer.shootOrchestrator.setEnableShooting(true),
                        () -> RobotContainer.shootOrchestrator.setEnableShooting(false)));

        // Configures auto builder
        AutoBuilder.configure(
                RobotContainer.poseSensorFusion::getEstimatedPosition, // Robot pose supplier
                RobotContainer.poseSensorFusion
                        ::setToPose, // Method to reset odometry (will be called if your auto has a starting
                // pose)
                RobotContainer.drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE

                // Method that will drive the robot given ROBOT RELATIVE speeds
                (speeds, feedforwards) -> AutoControlModifier.getDefault()
                        .drive(
                                speeds,
                                feedforwards.robotRelativeForcesXNewtons(),
                                feedforwards.robotRelativeForcesYNewtons()),
                Constants.Swerve.PP_DRIVE_CONTROLLER,
                config,

                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                () -> {
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },

                // Reference to this subsystem to set requirements
                RobotContainer.drivetrain);

        PathPlannerLogging.setLogActivePathCallback(activePath -> {
            RobotContainer.fieldStateTracker.getField().getObject("Trajectory").setPoses(activePath);
            if (activePath.isEmpty()) {
                RobotContainer.fieldStateTracker
                        .getField()
                        .getObject("Setpoint")
                        .setPoses();
            }
            Logger.recordOutput("Auto/Trajectory", activePath.toArray(Pose2d[]::new));
        });
        PathPlannerLogging.setLogTargetPoseCallback(targetPose -> {
            RobotContainer.fieldStateTracker.getField().getObject("Setpoint").setPose(targetPose);
            Logger.recordOutput("Auto/TrajectorySetpoint", targetPose);
        });
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }
}
