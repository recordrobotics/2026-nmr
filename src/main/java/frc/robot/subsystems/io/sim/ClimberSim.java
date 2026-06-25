package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.real.ClimberReal;
import frc.robot.utils.SimpleMath;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class ClimberSim extends ClimberReal {

    private final double periodicDt;

    private AbstractDriveTrainSimulation drivetrainSim;

    private final ElevatorSim physicsSimNotSupportingRobot = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            Constants.Climber.GEAR_RATIO,
            Constants.Climber.CARRIAGE_MASS_KG,
            Constants.Climber.SPROCKET_EFFECTIVE_RADIUS,
            0,
            Constants.Climber.MAX_HEIGHT_METERS,
            true,
            0,
            0.0003,
            0.0003);

    private final ElevatorSim physicsSimSupportingRobot = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            Constants.Climber.GEAR_RATIO,
            Constants.Frame.ROBOT_MASS_KG - Constants.Climber.CARRIAGE_MASS_KG,
            Constants.Climber.SPROCKET_EFFECTIVE_RADIUS,
            0,
            Constants.Climber.MAX_HEIGHT_METERS,
            true,
            0,
            0.0003,
            0.0003);

    private ElevatorSim currentPhysicsSim = physicsSimNotSupportingRobot;

    public ClimberSim(double periodicDt, AbstractDriveTrainSimulation drivetrainSim) {
        this.periodicDt = periodicDt;
        this.drivetrainSim = drivetrainSim;

        motor.getSimState().Orientation = ChassisReference.Clockwise_Positive; // correct

        RobotContainer.pdp.registerSimDevice(12, motor.getSimState()::getSupplyCurrentMeasure);
    }

    @Override
    public void setPosition(double newValue) {
        // Reset internal sim state
        currentPhysicsSim.setState(newValue, 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        updateRotor();

        super.setPosition(newValue);
    }

    public static Pose3d getSimulatedClimberPose(Pose2d robotPose, double climberHeightMeters) {
        Pose2d climberPose2d = robotPose.transformBy(Constants.Climber.ROBOT_TO_CLIMBER_OFFSET);
        return SimpleMath.withHeight(climberPose2d, climberHeightMeters + Constants.Climber.CLIMBER_BASE_HEIGHT_METERS);
    }

    public static boolean isWithinDistanceOfClimbing(
            double distanceMeters, Pose2d robotPose, double climberHeightMeters) {
        Pose3d climberPose = getSimulatedClimberPose(robotPose, climberHeightMeters);
        for (Translation3d towerEndPose : Constants.Climber.END_OF_TOWER_POSITIONS) {
            if (climberPose.getTranslation().getDistance(towerEndPose) < distanceMeters) {
                return true;
            }
        }
        return false;
    }

    private void updateRotor() {
        motor.getSimState()
                .setRawRotorPosition(currentPhysicsSim.getPositionMeters() / Constants.Climber.METERS_PER_ROTATION);
        motor.getSimState()
                .setRotorVelocity(
                        currentPhysicsSim.getVelocityMetersPerSecond() / Constants.Climber.METERS_PER_ROTATION);
    }

    @Override
    public void simulationPeriodic() {
        if (isWithinDistanceOfClimbing(
                        Constants.Climber.END_OF_TOWER_POSITION_TOLERANCE,
                        drivetrainSim.getSimulatedDriveTrainPose(),
                        currentPhysicsSim.getPositionMeters())
                && currentPhysicsSim == physicsSimNotSupportingRobot) {
            currentPhysicsSim = physicsSimSupportingRobot;
            currentPhysicsSim.setState(
                    physicsSimNotSupportingRobot.getPositionMeters(),
                    physicsSimNotSupportingRobot.getVelocityMetersPerSecond());
        } else if (!isWithinDistanceOfClimbing(
                        Constants.Climber.END_OF_TOWER_POSITION_TOLERANCE,
                        drivetrainSim.getSimulatedDriveTrainPose(),
                        currentPhysicsSim.getPositionMeters())
                && currentPhysicsSim == physicsSimSupportingRobot) {
            currentPhysicsSim = physicsSimNotSupportingRobot;
            currentPhysicsSim.setState(
                    physicsSimSupportingRobot.getPositionMeters(),
                    physicsSimSupportingRobot.getVelocityMetersPerSecond());
        }

        motor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = motor.getSimState().getMotorVoltage();

        currentPhysicsSim.setInputVoltage(motorVoltage);
        currentPhysicsSim.update(periodicDt);

        updateRotor();
    }
}
