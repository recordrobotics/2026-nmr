package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.camera.PhysicalCamera;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map.Entry;
import java.util.Set;
import java.util.function.Supplier;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.LoggedPowerDistribution;

public final class PowerDistributionPanel extends SubsystemBase {

    private static final Current RADIO_CURRENT = Amps.of(0.5);

    private final PDPSim pdpSim;
    private final HashMap<Integer, Supplier<Current>> simCurrentSuppliers;
    private final Set<Supplier<Current>> simMiniPdpDeviceCurrentSuppliers;

    public PowerDistributionPanel() {
        if (RobotBase.isSimulation()) {
            pdpSim = new PDPSim(1);
            simCurrentSuppliers = new HashMap<>();
            simMiniPdpDeviceCurrentSuppliers = new HashSet<>();

            SimulatedBattery.addElectricalAppliances(this::getMiniPDPCurrent);

            // RoboRIO
            registerSimDevice(20, RobotController::getMeasureInputCurrent);
            SimulatedBattery.addElectricalAppliances(RobotController::getMeasureInputCurrent);

            // Radio
            registerSimDevice(21, () -> RADIO_CURRENT);
            SimulatedBattery.addElectricalAppliances(() -> RADIO_CURRENT);

            // VRM
            registerSimDevice(22, this::getVRMCurrent);
            SimulatedBattery.addElectricalAppliances(this::getVRMCurrent);

            // VRM (LEDs)
            registerSimDevice(23, this::getLEDsCurrent);
            SimulatedBattery.addElectricalAppliances(this::getLEDsCurrent);

            // OPI
            for (int i = 0; i < 3; i++) { // 3 OPIs
                registerSimMiniPdpDevice(() -> Amps.of(20.0 / 12.0)); // 20W at 12V
            }

            // LL4
            registerSimMiniPdpDevice(() -> PhysicalCamera.LIMELIGHT_4.currentDraw);
        } else {
            pdpSim = null;
            simCurrentSuppliers = null;
            simMiniPdpDeviceCurrentSuppliers = null;
        }
    }

    public static void setupLogging() {
        LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
    }

    public void registerSimDevice(int channel, Supplier<Current> currentSupplier) {
        if (RobotBase.isSimulation()) {
            simCurrentSuppliers.put(channel, currentSupplier);
        }
    }

    public void registerSimMiniPdpDevice(Supplier<Current> currentSupplier) {
        if (RobotBase.isSimulation()) {
            simMiniPdpDeviceCurrentSuppliers.add(currentSupplier);
        }
    }

    public Current getVRMCurrent() {
        if (!RobotBase.isSimulation()) {
            return Amps.of(0);
        }

        return PhysicalCamera.LIMELIGHT_2.currentDraw.plus(PhysicalCamera.LIMELIGHT_3G.currentDraw);
    }

    public Current getLEDsCurrent() {
        return Amps.of(2.0);
    }

    public Current getMiniPDPCurrent() {
        if (RobotBase.isSimulation()) {
            double amps = 0;
            for (Supplier<Current> supplier : simMiniPdpDeviceCurrentSuppliers) {
                amps += supplier.get().in(Amps);
            }
            return Amps.of(amps);
        } else {
            return Amps.of(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        pdpSim.setInitialized(true);
        pdpSim.setVoltage(RobotController.getBatteryVoltage());
        pdpSim.setTemperature(30);

        for (Entry<Integer, Supplier<Current>> entry : simCurrentSuppliers.entrySet()) {
            pdpSim.setCurrent(entry.getKey(), entry.getValue().get().in(Amps));
        }

        pdpSim.setCurrent(8, getMiniPDPCurrent().in(Amps));
    }
}
