package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.Milliamps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.real.SwerveModuleReal;
import frc.robot.utils.ModuleConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class SwerveModuleSim extends SwerveModuleReal {

    private final SimDouble absoluteTurningMotorEncoderPosition;

    public static class TalonFXMotorControllerSim implements SimulatedMotorController {
        public final int id;

        private final TalonFXSimState talonFXSimState;

        private AngularVelocity lastVelocity = RotationsPerSecond.of(0);

        public TalonFXMotorControllerSim(TalonFX talonFX) {
            this.id = talonFX.getDeviceID();
            this.talonFXSimState = talonFX.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            talonFXSimState.setRawRotorPosition(encoderAngle);
            talonFXSimState.setRotorVelocity(encoderVelocity);
            talonFXSimState.setRotorAcceleration(
                    (encoderVelocity.in(RotationsPerSecond) - lastVelocity.in(RotationsPerSecond))
                            / SimulatedArena.getSimulationDt().in(Seconds));
            lastVelocity = encoderVelocity;
            talonFXSimState.setSupplyVoltage(SimulatedBattery.ROBORIO_BATTERY.getBatteryVoltage());

            return talonFXSimState.getMotorVoltageMeasure();
        }
    }

    public SwerveModuleSim(
            SwerveModuleSimulation moduleSimulation,
            ModuleConstants m,
            int driveTrack,
            int turnTrack,
            int drivePdpChannel,
            int turnPdpChannel) {
        super(m, driveTrack, turnTrack);

        driveMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        turningMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;

        driveMotor.getSimState().setMotorType(MotorType.KrakenX60);
        turningMotor.getSimState().setMotorType(MotorType.KrakenX44);

        SimDeviceSim absoluteTurningMotorEncoderSim =
                new SimDeviceSim("DutyCycle:DutyCycleEncoder", m.absoluteTurningMotorEncoderChannel());
        absoluteTurningMotorEncoderPosition = absoluteTurningMotorEncoderSim.getDouble("Position");
        moduleSimulation.useDriveMotorController(new TalonFXMotorControllerSim(driveMotor));
        moduleSimulation.useSteerMotorController(new TalonFXMotorControllerSim(turningMotor));

        RobotContainer.pdp.registerSimDevice(drivePdpChannel, driveMotor.getSimState()::getSupplyCurrentMeasure);
        RobotContainer.pdp.registerSimDevice(turnPdpChannel, turningMotor.getSimState()::getSupplyCurrentMeasure);
        RobotContainer.pdp.registerSimMiniPdpDevice(() -> Milliamps.of(50));
    }

    @Override
    public void setTurnMechanismPosition(double newValue) {
        /* don't reset position in simulation */
    }

    @Override
    public void simulationPeriodic() {
        absoluteTurningMotorEncoderPosition.set(turningMotor.getPosition().getValueAsDouble());
    }
}
