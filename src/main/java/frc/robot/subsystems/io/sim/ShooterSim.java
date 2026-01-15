package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel;
import frc.robot.subsystems.io.real.ShooterReal;
import frc.robot.utils.SimpleMath;
import java.util.Arrays;

public class ShooterSim extends ShooterReal {

    private static final double FLYWHEEL_SHOOT_VOLTAGE_MULTIPLIER = 0.88;

    private final double periodicDt;

    private final DCMotor flywheelMotor = DCMotor.getKrakenX60(2);
    private final DCMotor hoodMotor = DCMotor.getKrakenX44(1);

    private final SingleJointedArmSim hoodSimModel = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(hoodMotor, 0.007440336, Constants.Shooter.HOOD_GEAR_RATIO),
            hoodMotor,
            Constants.Shooter.HOOD_GEAR_RATIO,
            0.107361364429, // distance from hood pinion to center of mass
            Constants.Shooter.HOOD_MIN_POSITION_RADIANS
                    + Units.rotationsToRadians(Constants.Shooter.HOOD_GRAVITY_POSITION_OFFSET_ROTATIONS),
            Constants.Shooter.HOOD_MAX_POSITION_RADIANS
                    + Units.rotationsToRadians(Constants.Shooter.HOOD_GRAVITY_POSITION_OFFSET_ROTATIONS),
            true,
            Constants.Shooter.HOOD_STARTING_POSITION_RADIANS
                    + Units.rotationsToRadians(Constants.Shooter.HOOD_GRAVITY_POSITION_OFFSET_ROTATIONS),
            0.0,
            0.0);

    private final DCMotorSim flywheelSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(flywheelMotor, 0.0006979201, Constants.Shooter.FLYWHEEL_GEAR_RATIO),
            flywheelMotor,
            0.0,
            0.0);

    public ShooterSim(double periodicDt) {
        this.periodicDt = periodicDt;

        flywheelGroup.getSimState(0).Orientation = ChassisReference.CounterClockwise_Positive;
        flywheelGroup.getSimState(1).Orientation = ChassisReference.Clockwise_Positive;
        hood.getSimState().Orientation = ChassisReference.Clockwise_Positive;

        flywheelGroup.getSimState(0).setMotorType(MotorType.KrakenX60);
        flywheelGroup.getSimState(1).setMotorType(MotorType.KrakenX60);
        hood.getSimState().setMotorType(MotorType.KrakenX44);

        RobotContainer.pdp.registerSimDevice(
                15, () -> flywheelGroup.getSimState(0).getSupplyCurrentMeasure());
        RobotContainer.pdp.registerSimDevice(
                16, () -> flywheelGroup.getSimState(1).getSupplyCurrentMeasure());
        RobotContainer.pdp.registerSimDevice(17, () -> hood.getSimState().getSupplyCurrentMeasure());
    }

    @Override
    public void setFlywheelPositionMeters(double newValue) {
        // Reset internal sim state
        flywheelSimModel.setState(Units.rotationsToRadians(newValue), 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        updateFlywheelRotor();

        super.setFlywheelPositionMeters(newValue);
    }

    @Override
    public void setHoodPositionRotations(double newValueRotations) {
        // Reset internal sim state
        hoodSimModel.setState(
                Units.rotationsToRadians(newValueRotations + Constants.Shooter.HOOD_GRAVITY_POSITION_OFFSET_ROTATIONS),
                0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        updateHoodRotor();

        super.setHoodPositionRotations(newValueRotations);
    }

    private void updateFlywheelRotor() {
        for (TalonFXSimState simState : flywheelGroup.getSimStates()) {
            simState.setRawRotorPosition(
                    Constants.Shooter.FLYWHEEL_GEAR_RATIO * flywheelSimModel.getAngularPositionRotations());
            simState.setRotorVelocity(Constants.Shooter.FLYWHEEL_GEAR_RATIO
                    * Units.radiansToRotations(flywheelSimModel.getAngularVelocityRadPerSec()));
            simState.setRotorAcceleration(Constants.Shooter.FLYWHEEL_GEAR_RATIO
                    * Units.radiansToRotations(flywheelSimModel.getAngularAccelerationRadPerSecSq()));
        }
    }

    private void updateHoodRotor() {
        hood.getSimState()
                .setRawRotorPosition(Constants.Shooter.HOOD_GEAR_RATIO
                                * Units.radiansToRotations(
                                        hoodSimModel.getAngleRads() - Constants.Shooter.HOOD_STARTING_POSITION_RADIANS)
                        - Constants.Shooter.HOOD_GRAVITY_POSITION_OFFSET_ROTATIONS);
        hood.getSimState()
                .setRotorVelocity(Constants.Shooter.HOOD_GEAR_RATIO
                        * Units.radiansToRotations(hoodSimModel.getVelocityRadPerSec()));
    }

    @Override
    public void simulationPeriodic() {
        for (TalonFXSimState simState : flywheelGroup.getSimStates()) {
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        }

        hood.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double hoodVoltage = hood.getSimState().getMotorVoltage();
        double flywheelVoltage = SimpleMath.average(Arrays.stream(flywheelGroup.getSimStates())
                        .mapToDouble(TalonFXSimState::getMotorVoltage)
                        .toArray())
                .orElse(0);

        hoodSimModel.setInputVoltage(hoodVoltage);
        hoodSimModel.update(periodicDt);

        flywheelSimModel.setInputVoltage(
                (RobotModel.getFuelManager().isShootingFuel() ? FLYWHEEL_SHOOT_VOLTAGE_MULTIPLIER : 1.0)
                        * flywheelVoltage);
        flywheelSimModel.update(periodicDt);

        updateHoodRotor();
        updateFlywheelRotor();
    }
}
