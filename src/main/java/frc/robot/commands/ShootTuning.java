package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterState;

public class ShootTuning extends SequentialCommandGroup {
    private static final double VELOCITY_STEP = (18 - 6) / 10.0;

    public ShootTuning() {
        addRequirements(RobotContainer.shooter);
        addCommands(new InstantCommand(
                () -> {
                    RobotContainer.shootOrchestrator.setEnableShooting(true);
                },
                RobotContainer.shooter));

        for (double velocity = 6; velocity <= 18; velocity += VELOCITY_STEP) {
            final double finalVelocity = velocity;
            addCommands(
                    new InstantCommand(
                            () -> {
                                RobotContainer.shootOrchestrator.setShooterOverride(new ShooterState(finalVelocity, 0));
                            },
                            RobotContainer.shooter),
                    new WaitUntilCommand(() ->
                            RobotContainer.feeder.isBottomBeamBroken() || RobotContainer.feeder.isTopBeamBroken()),
                    new WaitCommand(0.5));
        }

        addCommands(new InstantCommand(
                () -> {
                    RobotContainer.shootOrchestrator.setEnableShooting(false);
                    RobotContainer.shootOrchestrator.clearShooterOverride();
                },
                RobotContainer.shooter));
    }
}
