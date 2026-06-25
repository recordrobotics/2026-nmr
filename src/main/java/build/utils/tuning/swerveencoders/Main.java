package build.utils.tuning.swerveencoders;

import build.utils.FileUtils;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.utils.ConsoleLogger;
import java.io.File;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public final class Main {
    private static final int TEAM_NUMBER = 6731;

    private Main() {}

    @SuppressWarnings("java:S109")
    public static void main(String... args) {
        if (args.length == 0) {
            throw new IllegalArgumentException("Please provide the motor name as the first argument (e.g., 'kraken')");
        }

        WPIUtilJNI.checkMsvcRuntime();

        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("Failed to initialize. Terminating");
        }

        System.out.println("Reading config");
        File config = new File(FileUtils.getLaunchDirectory(), "src/main/deploy/swerve/motors.json");

        Set<Integer> encoderChannels;
        try {
            encoderChannels = ConfigHelper.getAllEncoderChannels(config);
            System.out.println("Configured encoder channels: " + encoderChannels);
        } catch (Exception e) {
            ConsoleLogger.logError(e);
            return;
        }

        System.out.println("Starting NetworkTables");

        try (NetworkTableInstance nt = NetworkTableInstance.create()) {
            nt.setServerTeam(TEAM_NUMBER);
            nt.startClient4("SwerveEncoderTuning");
            nt.startDSClient();

            while (!nt.isConnected()) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    ConsoleLogger.logError(e);
                    Thread.currentThread().interrupt();
                }
            }

            System.out.println("NetworkTables connected");

            Map<Integer, EncoderSubscription> subscriptions = new HashMap<>();
            for (int channel : encoderChannels) {
                subscriptions.put(channel, subscribeToEncoder(nt, channel));
            }

            Map<Integer, Double> values = new HashMap<>();

            for (Map.Entry<Integer, EncoderSubscription> entry : subscriptions.entrySet()) {
                values.put(entry.getKey(), entry.getValue().waitForValue());
            }

            System.out.println("Read encoder values:");
            for (Map.Entry<Integer, Double> entry : values.entrySet()) {
                System.out.println("Encoder " + entry.getKey() + ": " + entry.getValue());
            }

            try {
                ConfigHelper.setEncoderOffsets(config, values, args[0]);
                System.out.println("Updated config file: " + config.getAbsolutePath());
            } catch (Exception e) {
                ConsoleLogger.logError(e);
            }

            System.out.println("NetworkTables closed");
        }
    }

    private static class EncoderSubscription {
        private boolean hasValue = false;
        private double value = 0.0;

        private synchronized void update(double newValue) {
            hasValue = true;
            value = newValue;
        }

        @SuppressWarnings("java:S109")
        double waitForValue() {
            while (true) {
                synchronized (this) {
                    if (hasValue) {
                        return value;
                    }
                }
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    private static EncoderSubscription subscribeToEncoder(NetworkTableInstance nt, int encoderIndex) {
        EncoderSubscription sub = new EncoderSubscription();

        nt.addListener(nt.getEntry("/SmartDashboard/Encoder " + encoderIndex), EnumSet.of(Kind.kValueRemote), ev -> {
            System.out.println("Encoder " + encoderIndex + " value updated: " + ev.valueData.value.getDouble());
            sub.update(ev.valueData.value.getDouble());
        });

        return sub;
    }
}
