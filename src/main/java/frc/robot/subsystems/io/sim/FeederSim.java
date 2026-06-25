package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel;
import frc.robot.subsystems.RobotModel.FuelManager.LineSegment;
import frc.robot.subsystems.io.real.FeederReal;

public class FeederSim extends FeederReal {

    private static final double SHOOT_VOLTAGE_MULTIPLIER = 0.6;

    private static final LineSegment topBeambreakLine = new LineSegment(
            new Translation3d(0.186525, 0.254875, 0.324841), new Translation3d(0.186525, -0.000875, 0.324841));
    private static final LineSegment bottomBeambreakLine = new LineSegment(
            new Translation3d(0.157081, 0.254875, 0.230972), new Translation3d(0.157081, -0.000875, 0.230972));

    private final double periodicDt;

    private final DCMotor feederMotor = DCMotor.getKrakenX60(1);

    private final DCMotorSim feederSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(feederMotor, 0.0001243628, Constants.Feeder.GEAR_RATIO),
            feederMotor,
            0.0,
            0.0);

    private final DIOSim bottomBeambreakSim;
    private final DIOSim topBeambreakSim;

    public FeederSim(double periodicDt) {
        this.periodicDt = periodicDt;

        feeder.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        feeder.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

        bottomBeambreakSim = new DIOSim(bottomBeambreak);
        topBeambreakSim = new DIOSim(topBeambreak);
        bottomBeambreakSim.setIsInput(true);
        topBeambreakSim.setIsInput(true);

        RobotContainer.pdp.registerSimDevice(13, feeder.getSimState()::getSupplyCurrentMeasure);
    }

    @Override
    public void simulationPeriodic() {
        feeder.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double feederVoltage = feeder.getSimState().getMotorVoltage();

        feederSimModel.setInputVoltage(
                (RobotModel.getFuelManager().isFuelInFeeder() ? SHOOT_VOLTAGE_MULTIPLIER : 1.0) * feederVoltage);
        feederSimModel.update(periodicDt);

        feeder.getSimState()
                .setRawRotorPosition(Constants.Feeder.GEAR_RATIO * feederSimModel.getAngularPositionRotations());
        feeder.getSimState()
                .setRotorVelocity(Constants.Feeder.GEAR_RATIO
                        * Units.radiansToRotations(feederSimModel.getAngularVelocityRadPerSec()));
        feeder.getSimState()
                .setRotorAcceleration(Constants.Feeder.GEAR_RATIO
                        * Units.radiansToRotations(feederSimModel.getAngularAccelerationRadPerSecSq()));

        bottomBeambreakSim.setValue(!RobotModel.getFuelManager().hasFuelIntersecting(bottomBeambreakLine));
        topBeambreakSim.setValue(!RobotModel.getFuelManager().hasFuelIntersecting(topBeambreakLine));
    }

    public boolean isOuttaking() {
        return feeder.getVelocity().getValueAsDouble() >= Constants.Feeder.INTAKE_VELOCITY_RPS / 2.0;
    }

    @Override
    public void close() {
        feeder.close();
        bottomBeambreak.close();
        topBeambreak.close();
    }
}
