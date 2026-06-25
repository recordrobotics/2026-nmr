package build.utils.tuning.swerveencoders;

import build.utils.FileUtils;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public final class ConfigHelper {

    private static final String ENCODER_CHANNEL = "encoderChannel";

    private ConfigHelper() {}

    public static Set<Integer> getAllEncoderChannels(File configFile) throws FileNotFoundException {
        JsonNode obj = FileUtils.readJson(configFile);
        Set<Integer> encoderChannels = new HashSet<>();

        obj.properties().iterator().forEachRemaining(entry -> {
            JsonNode moduleConfig = entry.getValue();
            if (moduleConfig.has(ENCODER_CHANNEL)) {
                encoderChannels.add(moduleConfig.get(ENCODER_CHANNEL).asInt());
            }
        });

        return encoderChannels;
    }

    public static void setEncoderOffsets(File configFile, Map<Integer, Double> offsets, String motorName)
            throws FileNotFoundException {
        JsonNode obj = FileUtils.readJson(configFile);

        obj.properties().iterator().forEachRemaining(entry -> {
            JsonNode moduleConfig = entry.getValue();
            if (moduleConfig.has(ENCODER_CHANNEL)) {
                int channel = moduleConfig.get(ENCODER_CHANNEL).asInt();
                if (offsets.containsKey(channel)
                        && moduleConfig instanceof ObjectNode objectNode
                        && objectNode.get("encoderOffset") instanceof ObjectNode motorNode) {
                    if (motorNode.get(motorName) == null) {
                        System.out.println(
                                "No existing offset for motor " + motorName + " on channel " + channel + ", skipping");
                        return;
                    }
                    motorNode.put(motorName, offsets.get(channel));
                } else {
                    System.out.println("No offset for channel " + channel + ", skipping");
                }
            }
        });

        FileUtils.writeJson(configFile, obj, new ConfigPrettyPrinter());
    }
}
