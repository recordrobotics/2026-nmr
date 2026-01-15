package frc.robot.utils.maplesim;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public final class SimulatedBatteryFactory {

    public static final double ACTUAL_RESTING_BATTERY_VOLTAGE = 12.68;

    private SimulatedBatteryFactory() {}

    public static SimulatedBattery create() {
        return register(modify(new SimulatedBattery()));
    }

    public static SimulatedBattery modify(SimulatedBattery battery) {
        battery.setVoltage(ACTUAL_RESTING_BATTERY_VOLTAGE);
        battery.setDischargeRate(0.02 / 378.45);
        return battery;
    }

    public static SimulatedBattery register(SimulatedBattery battery) {
        SimulatedArena.getInstance().addBatterySimulation(battery);
        return battery;
    }
}
