package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel;
import frc.robot.subsystems.io.real.IndexerReal;

public class IndexerSim extends IndexerReal {

    private static final double FUEL_VOLTAGE_MULTIPLIER_A = 2677.5;
    private static final double FUEL_VOLTAGE_MULTIPLIER_B = 127.5;
    private static final double FUEL_VOLTAGE_MULTIPLIER_DIV = 21.0;

    private final double periodicDt;

    private final DCMotor indexerMotor = DCMotor.getKrakenX60(1);

    private final DCMotorSim indexerSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(indexerMotor, 0.0004429452, Constants.Indexer.GEAR_RATIO),
            indexerMotor,
            0.0,
            0.0);

    public IndexerSim(double periodicDt) {
        this.periodicDt = periodicDt;

        indexer.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        indexer.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

        RobotContainer.pdp.registerSimDevice(11, indexer.getSimState()::getSupplyCurrentMeasure);
    }

    public boolean isOuttaking() {
        return indexer.getVelocity().getValueAsDouble() >= Constants.Indexer.INTAKE_VELOCITY_RPS / 2.0;
    }

    @Override
    public void simulationPeriodic() {
        indexer.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double indexerVoltage = indexer.getSimState().getMotorVoltage();

        indexerSimModel.setInputVoltage(indexerVoltage
                * calculateVoltageMultiplier(RobotModel.getFuelManager().getFuelCount()));
        indexerSimModel.update(periodicDt);

        indexer
                .getSimState()
                .setRawRotorPosition(Constants.Indexer.GEAR_RATIO * indexerSimModel.getAngularPositionRotations());
        indexer
                .getSimState()
                .setRotorVelocity(Constants.Indexer.GEAR_RATIO
                        * Units.radiansToRotations(indexerSimModel.getAngularVelocityRadPerSec()));
        indexer
                .getSimState()
                .setRotorAcceleration(Constants.Indexer.GEAR_RATIO
                        * Units.radiansToRotations(indexerSimModel.getAngularAccelerationRadPerSecSq()));
    }

    private static double calculateVoltageMultiplier(int fuelCount) {
        return FUEL_VOLTAGE_MULTIPLIER_A / (fuelCount + FUEL_VOLTAGE_MULTIPLIER_B) / FUEL_VOLTAGE_MULTIPLIER_DIV;
    }
}
