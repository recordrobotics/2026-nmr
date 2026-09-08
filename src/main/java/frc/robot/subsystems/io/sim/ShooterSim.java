package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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

    private final DCMotorSim flywheelSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(flywheelMotor, 0.0006979201, Constants.Shooter.FLYWHEEL_GEAR_RATIO),
            flywheelMotor,
            0.0,
            0.0);

    public ShooterSim(double periodicDt) {
        this.periodicDt = periodicDt;

        flywheelGroup.getSimState(0).Orientation = ChassisReference.CounterClockwise_Positive;
        flywheelGroup.getSimState(1).Orientation = ChassisReference.Clockwise_Positive;

        flywheelGroup.getSimState(0).setMotorType(MotorType.KrakenX60);
        flywheelGroup.getSimState(1).setMotorType(MotorType.KrakenX60);

        RobotContainer.pdp.registerSimDevice(
                15, () -> flywheelGroup.getSimState(0).getSupplyCurrentMeasure());
        RobotContainer.pdp.registerSimDevice(
                16, () -> flywheelGroup.getSimState(1).getSupplyCurrentMeasure());
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

    @Override
    public void simulationPeriodic() {
        for (TalonFXSimState simState : flywheelGroup.getSimStates()) {
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        }

        double flywheelVoltage = SimpleMath.average(Arrays.stream(flywheelGroup.getSimStates())
                        .mapToDouble(TalonFXSimState::getMotorVoltage)
                        .toArray())
                .orElse(0);

        flywheelSimModel.setInputVoltage(
                (RobotModel.getFuelManager().isShootingFuel() ? FLYWHEEL_SHOOT_VOLTAGE_MULTIPLIER : 1.0)
                        * flywheelVoltage);
        flywheelSimModel.update(periodicDt);

        updateFlywheelRotor();
    }
}
