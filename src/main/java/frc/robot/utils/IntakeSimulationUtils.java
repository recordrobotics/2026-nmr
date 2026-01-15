package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public final class IntakeSimulationUtils {
    private IntakeSimulationUtils() {}

    @SuppressWarnings("java:S109")
    public static Rectangle getIntakeRectangle(
            AbstractDriveTrainSimulation driveTrainSimulation, double width, double lengthExtended, IntakeSide side) {
        final Rectangle intakeRectangle = new Rectangle(width, lengthExtended);
        intakeRectangle.rotate(
                switch (side) {
                    case LEFT, RIGHT -> 0;
                    case FRONT, BACK -> Math.toRadians(90);
                });
        final double distanceTransformed = lengthExtended / 2 - 0.01;
        intakeRectangle.translate(
                switch (side) {
                    case LEFT ->
                        new Vector2(0, driveTrainSimulation.config.bumperWidthY.in(Meters) / 2 + distanceTransformed);
                    case RIGHT ->
                        new Vector2(0, -driveTrainSimulation.config.bumperWidthY.in(Meters) / 2 - distanceTransformed);
                    case FRONT ->
                        new Vector2(driveTrainSimulation.config.bumperLengthX.in(Meters) / 2 + distanceTransformed, 0);
                    case BACK ->
                        new Vector2(
                                -driveTrainSimulation.config.bumperLengthX.in(Meters) / 2 - distanceTransformed / 2, 0);
                });

        return intakeRectangle;
    }
}
