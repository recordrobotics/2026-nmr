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
import frc.robot.subsystems.io.real.SpindexerReal;

public class SpindexerSim extends SpindexerReal {

    private static final double FUEL_VOLTAGE_MULTIPLIER_A = 2677.5;
    private static final double FUEL_VOLTAGE_MULTIPLIER_B = 127.5;
    private static final double FUEL_VOLTAGE_MULTIPLIER_DIV = 21.0;

    private final double periodicDt;

    private final DCMotor spindexerMotor = DCMotor.getKrakenX60(1);

    private final DCMotorSim spindexerSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(spindexerMotor, 0.0004429452, Constants.Spindexer.GEAR_RATIO),
            spindexerMotor,
            0.0,
            0.0);

    public SpindexerSim(double periodicDt) {
        this.periodicDt = periodicDt;

        spindexer.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        spindexer.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

        RobotContainer.pdp.registerSimDevice(11, spindexer.getSimState()::getSupplyCurrentMeasure);
    }

    public boolean isOuttaking() {
        return spindexer.getVelocity().getValueAsDouble() >= Constants.Spindexer.INTAKE_VELOCITY_RPS / 2.0;
    }

    @Override
    public void simulationPeriodic() {
        spindexer.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double spindexerVoltage = spindexer.getSimState().getMotorVoltage();

        spindexerSimModel.setInputVoltage(spindexerVoltage
                * calculateVoltageMultiplier(RobotModel.getFuelManager().getFuelCount()));
        spindexerSimModel.update(periodicDt);

        spindexer
                .getSimState()
                .setRawRotorPosition(Constants.Spindexer.GEAR_RATIO * spindexerSimModel.getAngularPositionRotations());
        spindexer
                .getSimState()
                .setRotorVelocity(Constants.Spindexer.GEAR_RATIO
                        * Units.radiansToRotations(spindexerSimModel.getAngularVelocityRadPerSec()));
        spindexer
                .getSimState()
                .setRotorAcceleration(Constants.Spindexer.GEAR_RATIO
                        * Units.radiansToRotations(spindexerSimModel.getAngularAccelerationRadPerSecSq()));
    }

    private static double calculateVoltageMultiplier(int fuelCount) {
        return FUEL_VOLTAGE_MULTIPLIER_A / (fuelCount + FUEL_VOLTAGE_MULTIPLIER_B) / FUEL_VOLTAGE_MULTIPLIER_DIV;
    }
}
