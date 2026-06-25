package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.io.real.TurretReal;

public class TurretSim extends TurretReal {

    private final double periodicDt;

    private final DCMotor turretMotor = DCMotor.getKrakenX44(1);

    private final DCMotorSim turretSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turretMotor, 0.0962647197, Constants.Turret.GEAR_RATIO),
            turretMotor,
            0.0,
            0.0);

    private final MagneticLimitSwitch frontLeftLimitSwitchSimModel = new MagneticLimitSwitch(
            Constants.Turret.FRONT_LEFT_LIMIT_SWITCH_POSITION_RADIANS,
            Constants.Turret.MAGNETIC_LIMIT_SWITCH_TRIGGER_ANGLE_RAD,
            Constants.Turret.MAGNETIC_LIMIT_SWITCH_DETRIGGER_ANGLE_RAD);
    private final MagneticLimitSwitch backLeftLimitSwitchSimModel = new MagneticLimitSwitch(
            Constants.Turret.BACK_LEFT_LIMIT_SWITCH_POSITION_RADIANS,
            Constants.Turret.MAGNETIC_LIMIT_SWITCH_TRIGGER_ANGLE_RAD,
            Constants.Turret.MAGNETIC_LIMIT_SWITCH_DETRIGGER_ANGLE_RAD);
    private final MagneticLimitSwitch backRightLimitSwitchSimModel = new MagneticLimitSwitch(
            Constants.Turret.BACK_RIGHT_LIMIT_SWITCH_POSITION_RADIANS,
            Constants.Turret.MAGNETIC_LIMIT_SWITCH_TRIGGER_ANGLE_RAD,
            Constants.Turret.MAGNETIC_LIMIT_SWITCH_DETRIGGER_ANGLE_RAD);

    private final DIOSim frontLeftLimitSwitchSim;
    private final DIOSim backLeftLimitSwitchSim;
    private final DIOSim backRightLimitSwitchSim;

    public TurretSim(double periodicDt) {
        this.periodicDt = periodicDt;

        turret.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        turret.getSimState().setMotorType(MotorType.KrakenX44);

        frontLeftLimitSwitchSim = new DIOSim(frontLeftLimitSwitch);
        backLeftLimitSwitchSim = new DIOSim(backLeftLimitSwitch);
        backRightLimitSwitchSim = new DIOSim(backRightLimitSwitch);
        frontLeftLimitSwitchSim.setIsInput(true);
        backLeftLimitSwitchSim.setIsInput(true);
        backRightLimitSwitchSim.setIsInput(true);

        RobotContainer.pdp.registerSimDevice(14, turret.getSimState()::getSupplyCurrentMeasure);
    }

    @Override
    public void setPositionRotations(double newValue) {
        // reset internal sim state
        turretSimModel.setState(Units.rotationsToRadians(newValue), 0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        updateRotor();

        super.setPositionRotations(newValue);

        updateLimitSwitches();
    }

    private double getSpringVoltage() {
        double pos = turret.getPosition().getValueAsDouble();
        if (pos > Constants.Turret.TURRET_SPRING_LOW_START_POS) return Constants.Turret.TURRET_SPRING_LOW_VOLTS;
        if (pos > Constants.Turret.TURRET_SPRING_HIGH_START_POS) return Constants.Turret.TURRET_SPRING_HIGH_VOLTS;

        if (pos < Constants.Turret.TURRET_SPRING_LOW_START_NEG) return -Constants.Turret.TURRET_SPRING_LOW_VOLTS;
        if (pos < Constants.Turret.TURRET_SPRING_HIGH_START_NEG) return -Constants.Turret.TURRET_SPRING_HIGH_VOLTS;

        return 0;
    }

    private void updateRotor() {
        turret.getSimState()
                .setRawRotorPosition(Constants.Turret.GEAR_RATIO * turretSimModel.getAngularPositionRotations());
        turret.getSimState()
                .setRotorVelocity(Constants.Turret.GEAR_RATIO
                        * Units.radiansToRotations(turretSimModel.getAngularVelocityRadPerSec()));
        turret.getSimState()
                .setRotorAcceleration(Constants.Turret.GEAR_RATIO
                        * Units.radiansToRotations(turretSimModel.getAngularAccelerationRadPerSecSq()));
    }

    @Override
    public void simulationPeriodic() {
        turret.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double turretVoltage = turret.getSimState().getMotorVoltage();

        turretSimModel.setInputVoltage(turretVoltage - getSpringVoltage());
        turretSimModel.update(periodicDt);

        updateRotor();

        updateLimitSwitches();
    }

    private void updateLimitSwitches() {
        double turretPositionRad = Units.rotationsToRadians(
                turretSimModel.getAngularPositionRotations() + Turret.MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS);

        // Wrap turret position within one rotation
        turretPositionRad = MathUtil.inputModulus(
                turretPositionRad - Constants.Turret.TURRET_MAGNET_OFFSET_ANGLE_RAD, 0, Math.PI * 2);

        frontLeftLimitSwitchSimModel.update(turretPositionRad);
        backLeftLimitSwitchSimModel.update(turretPositionRad);
        backRightLimitSwitchSimModel.update(turretPositionRad);

        frontLeftLimitSwitchSim.setValue(!frontLeftLimitSwitchSimModel.isTriggered());
        backLeftLimitSwitchSim.setValue(!backLeftLimitSwitchSimModel.isTriggered());
        backRightLimitSwitchSim.setValue(!backRightLimitSwitchSimModel.isTriggered());
    }

    private static final class MagneticLimitSwitch {
        private final double positionRad;
        private final double triggerAngleRad;
        private final double detriggerAngleRad;

        private boolean triggered = false;

        private MagneticLimitSwitch(double positionRad, double triggerAngleRad, double detriggerAngleRad) {
            this.positionRad = positionRad;
            this.triggerAngleRad = triggerAngleRad;
            this.detriggerAngleRad = detriggerAngleRad;
        }

        private boolean update(double turretPositionRad) {
            double angleDifference = Math.abs(turretPositionRad - positionRad);

            if (!triggered && angleDifference <= triggerAngleRad) {
                triggered = true;
            } else if (triggered && angleDifference >= detriggerAngleRad) {
                triggered = false;
            }

            return triggered;
        }

        private boolean isTriggered() {
            return triggered;
        }
    }
}
