package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel;
import frc.robot.subsystems.RobotModel.FuelManager;
import frc.robot.subsystems.io.real.IntakeReal;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.IntakeSimulationUtils;
import frc.robot.utils.SimpleMath;
import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;
import org.dyn4j.collision.CollisionBody;
import org.dyn4j.collision.Fixture;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.contact.Contact;
import org.dyn4j.dynamics.contact.SolvedContact;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.ContactCollisionData;
import org.dyn4j.world.World;
import org.dyn4j.world.listener.ContactListener;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

public class IntakeSim extends IntakeReal {

    private static final double FUEL_VOLTAGE_MULTIPLIER_A = 16.73759;
    private static final double FUEL_VOLTAGE_MULTIPLIER_B = 4.1844;
    private static final double FUEL_VOLTAGE_MULTIPLIER_DIV = 4.0;

    private static final Distance WIDTH = Inches.of(21.25);
    private static final Distance LENGTH_EXTENDED = Inches.of(8.419554);
    private static final Translation3d ROLLER_ORIGIN = new Translation3d(-0.594994, 0.329377, 0.163443);

    private static final double EJECT_BPS = 4.5;

    private final double periodicDt;

    private final DCMotor armMotor = DCMotor.getKrakenX44(2);
    private final DCMotor wheelMotor = DCMotor.getKrakenX44(1);

    private final SingleJointedArmSim armSimModel = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(armMotor, 0.2943129061, Constants.Intake.ARM_GEAR_RATIO),
            armMotor,
            Constants.Intake.ARM_GEAR_RATIO,
            0.253046091813, // distance from axis of rotation to center of mass
            Constants.Intake.ARM_DOWN_POSITION_RADIANS
                    + Units.rotationsToRadians(Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_ROTATIONS),
            Constants.Intake.ARM_MAX_POSITION_RADIANS
                    + Units.rotationsToRadians(Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_ROTATIONS),
            true,
            Constants.Intake.ARM_STARTING_POSITION_RADIANS
                    + Units.rotationsToRadians(Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_ROTATIONS),
            0.0,
            0.0);

    private final DCMotorSim wheelSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(wheelMotor, 0.0000819391, Constants.Intake.WHEEL_GEAR_RATIO),
            wheelMotor,
            0.0,
            0.0);

    private final IntakeSimulation intakeSimulation;

    private double lastEjectTime = 0.0;

    private AtomicInteger rollerFuelCount = new AtomicInteger(0);

    private double lastIntakeDamagedTime = 0.0;
    private boolean intakeDamageStopped = true;

    public IntakeSim(double periodicDt, AbstractDriveTrainSimulation drivetrainSim) {
        this.periodicDt = periodicDt;

        armGroup.getSimState(0).Orientation =
                ChassisReference.CounterClockwise_Positive; // Left of intake, right of robot // Arm up is positive
        armGroup.getSimState(1).Orientation =
                ChassisReference.Clockwise_Positive; // Right of intake, left of robot // Arm up is positive

        armGroup.getSimState(0).setMotorType(MotorType.KrakenX44);
        armGroup.getSimState(1).setMotorType(MotorType.KrakenX44);

        wheel.getSimState().Orientation = ChassisReference.Clockwise_Positive; // Positive is intake, negative is eject
        wheel.getSimState().setMotorType(MotorType.KrakenX44);

        Rectangle intakeRect = IntakeSimulationUtils.getIntakeRectangle(
                drivetrainSim, WIDTH.in(Meters), LENGTH_EXTENDED.in(Meters), IntakeSimulation.IntakeSide.BACK);

        intakeSimulation = new IntakeSimulation("Fuel", drivetrainSim, intakeRect, FuelManager.getNodeCount());
        intakeSimulation.setCustomIntakeCondition(gm -> {
            boolean passed = wheel.getVelocity().getValueAsDouble() > 0.2;
            if (passed) {
                try {
                    RobotModel.getFuelManager().intakeFuel(gm.getPose3d(), fuel -> {
                        if (new Translation2d(
                                                fuel.getPose().getX(),
                                                fuel.getPose().getZ())
                                        .getDistance(new Translation2d(ROLLER_ORIGIN.getX(), ROLLER_ORIGIN.getZ()))
                                > Constants.Intake.ROLLER_DIAMETER.in(Meters) / 2.0
                                        + Constants.Game.FUEL_DIAMETER_METERS / 2.0
                                        + 0.05) {
                            rollerFuelCount.decrementAndGet();
                            return true;
                        }
                        return false;
                    });
                    rollerFuelCount.incrementAndGet();
                } catch (IllegalStateException e) {
                    return false; // no available intake nodes, so fail the intake condition instead of crashing
                }
            }
            return passed;
        });

        try {
            Field physicsWorldField = SimulatedArena.class.getDeclaredField("physicsWorld");
            physicsWorldField.setAccessible(true);
            World<Body> physicsWorld = (World<Body>) physicsWorldField.get(SimulatedArena.getInstance());
            physicsWorld.addContactListener(new BreakingContactListener());
        } catch (NoSuchFieldException | IllegalAccessException e) {
            ConsoleLogger.logError("Failed to register intake contact listener", e);
        }

        RobotContainer.pdp.registerSimDevice(9, () -> armGroup.getSimState(0).getSupplyCurrentMeasure());
        RobotContainer.pdp.registerSimDevice(10, () -> armGroup.getSimState(1).getSupplyCurrentMeasure());
        RobotContainer.pdp.registerSimDevice(18, () -> wheel.getSimState().getSupplyCurrentMeasure());
    }

    public IntakeSimulation getIntakeSimulation() {
        return intakeSimulation;
    }

    @Override
    public void setArmPositionRotations(double newValue) {
        // Reset internal sim state
        armSimModel.setState(
                Units.rotationsToRadians(newValue)
                        + Units.rotationsToRadians(Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_ROTATIONS),
                0);

        // Update raw rotor position to match internal sim state (has to be called before setPosition to
        // have correct offset)
        updateArmRotor();

        super.setArmPositionRotations(newValue);
    }

    @Override
    public void simulationPeriodic() {
        updateMotorSimulations();
        handleIntakeSimulation();

        Logger.recordOutput("Intake/SimCount", rollerFuelCount.get());

        if (Timer.getTimestamp() - lastIntakeDamagedTime < 0.1) {
            intakeDamageStopped = false;
            Logger.recordOutput("Intake/Damaged", true);
            RobotContainer.getControl().vibrate(RumbleType.kBothRumble, 1);
        } else if (!intakeDamageStopped) {
            intakeDamageStopped = true;
            Logger.recordOutput("Intake/Damaged", false);
            RobotContainer.getControl().vibrate(RumbleType.kBothRumble, 0);
        }
    }

    private void updateArmRotor() {
        for (TalonFXSimState simState : armGroup.getSimStates()) {
            simState.setRawRotorPosition(Constants.Intake.ARM_GEAR_RATIO
                    * Units.radiansToRotations(armSimModel.getAngleRads()
                            - Units.rotationsToRadians(Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_ROTATIONS)
                            - Constants.Intake.ARM_STARTING_POSITION_RADIANS));
            simState.setRotorVelocity(
                    Constants.Intake.ARM_GEAR_RATIO * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));
        }
    }

    private void updateMotorSimulations() {
        for (TalonFXSimState simState : armGroup.getSimStates()) {
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        }

        wheel.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

        double wheelVoltage = wheel.getSimState().getMotorVoltage();
        double armVoltage = SimpleMath.average(Arrays.stream(armGroup.getSimStates())
                        .mapToDouble(TalonFXSimState::getMotorVoltage)
                        .toArray())
                .orElse(0);

        wheelSimModel.setInputVoltage(wheelVoltage * calculateVoltageMultiplier(rollerFuelCount.get()));
        wheelSimModel.update(periodicDt);

        armSimModel.setInputVoltage(armVoltage);
        armSimModel.update(periodicDt);

        updateArmRotor();

        wheel.getSimState()
                .setRawRotorPosition(Constants.Intake.WHEEL_GEAR_RATIO * wheelSimModel.getAngularPositionRotations());
        wheel.getSimState()
                .setRotorVelocity(Constants.Intake.WHEEL_GEAR_RATIO
                        * Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec()));
        wheel.getSimState()
                .setRotorAcceleration(Constants.Intake.WHEEL_GEAR_RATIO
                        * Units.radiansToRotations(wheelSimModel.getAngularAccelerationRadPerSecSq()));
    }

    public boolean isExtendingHopperSpaceAvailable() {
        return Units.rotationsToRadians(armGroup.getAveragePosition())
                < Constants.Intake.ARM_DOWN_POSITION_RADIANS + Units.degreesToRadians(5.0);
    }

    private void handleIntakeSimulation() {
        if (Units.rotationsToRadians(armGroup.getAveragePosition())
                < Constants.Intake.ARM_DOWN_POSITION_RADIANS + Units.degreesToRadians(5.0)) {
            intakeSimulation.startIntake();
            if (wheel.getVelocity().getValueAsDouble() < Constants.Intake.WHEEL_EJECT_VELOCITY_MPS + 2.0) {
                double timeSinceLastEject = Timer.getTimestamp() - lastEjectTime;
                if (timeSinceLastEject > 1.0 / EJECT_BPS) {
                    RobotModel.getFuelManager().ejectFuel().ifPresent(fuel -> {
                        rollerFuelCount.incrementAndGet();
                        fuel.rotateAround(
                                () -> ROLLER_ORIGIN,
                                () -> new Translation3d(0, 1, 0),
                                new Translation2d(
                                                fuel.getPose().getX(),
                                                fuel.getPose().getZ())
                                        .getDistance(new Translation2d(ROLLER_ORIGIN.getX(), ROLLER_ORIGIN.getZ())),
                                () -> Math.PI / 2,
                                () -> -wheelSimModel.getAngularVelocityRadPerSec(),
                                () -> {
                                    rollerFuelCount.decrementAndGet();
                                    RobotModel.getFuelManager()
                                            .toProjectile(
                                                    fuel, RobotContainer.drivetrain.getSwerveDriveSimulation(), null);
                                });
                    });
                    lastEjectTime = Timer.getTimestamp();
                }
            }
        } else {
            intakeSimulation.stopIntake();
        }

        intakeSimulation.setGamePiecesCount(RobotModel.getFuelManager().getFuelCount());
    }

    private static double calculateVoltageMultiplier(int fuelCount) {
        return FUEL_VOLTAGE_MULTIPLIER_A / (fuelCount + FUEL_VOLTAGE_MULTIPLIER_B) / FUEL_VOLTAGE_MULTIPLIER_DIV;
    }

    public final class BreakingContactListener implements ContactListener<Body> {
        @Override
        public void postSolve(ContactCollisionData<Body> collision, SolvedContact contact) {
            if (!intakeSimulation.isRunning()) return;

            final CollisionBody<?> collisionBody1 = collision.getBody1(), collisionBody2 = collision.getBody2();
            final Fixture fixture1 = collision.getFixture1(), fixture2 = collision.getFixture2();

            if (collisionBody1 instanceof Body body1 && collisionBody2 instanceof Body body2) {
                if (body1.getMass().getType() == MassType.INFINITE && fixture2 == IntakeSim.this.intakeSimulation) {
                    processContact(contact, body2);
                } else if (body2.getMass().getType() == MassType.INFINITE
                        && fixture1 == IntakeSim.this.intakeSimulation) {
                    processContact(contact, body1);
                }
            }
        }

        private void processContact(SolvedContact contact, Body body) {
            Vector2 contactPoint = contact.getPoint();
            Vector2 r = contactPoint.copy().subtract(body.getWorldCenter());

            Vector2 vLinear = body.getLinearVelocity();
            double omega = body.getAngularVelocity();

            Vector2 vAngular = new Vector2(-r.y * omega, r.x * omega);
            Vector2 vContact = vLinear.copy().add(vAngular);

            double contactSpeed = vContact.getMagnitude();
            if (contactSpeed > 0.3) {
                lastIntakeDamagedTime = Timer.getTimestamp();
            }
        }

        /* functions not used */
        @Override
        public void persist(ContactCollisionData<Body> collision, Contact oldContact, Contact newContact) {}

        @Override
        public void end(ContactCollisionData<Body> collision, Contact contact) {}

        @Override
        public void destroyed(ContactCollisionData<Body> collision, Contact contact) {}

        @Override
        public void collision(ContactCollisionData<Body> collision) {}

        @Override
        public void preSolve(ContactCollisionData<Body> collision, Contact contact) {}

        @Override
        public void begin(ContactCollisionData<Body> collision, Contact contact) {}
    }
}
