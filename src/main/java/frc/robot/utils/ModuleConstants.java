package frc.robot.utils;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.utils.wrappers.Translation2d;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

/**
 * holds all constants that are SPECIFIC to ONE swerve module
 *
 * @param driveMotorChannel drive motor port
 * @param turningMotorChannel turn motor port
 * @param absoluteTurningMotorEncoderChannel abs turn motor encoder port
 * @param turningEncoderOffset offset of the abs turn encoder at a set starting position (which we
 *     found through manually testing)
 * @param wheelLocation Translation2d object of where the wheel is relative to robot frame
 * @param wheelDiameter Diameter of the wheel
 */
public record ModuleConstants(
        int driveMotorChannel,
        int turningMotorChannel,
        int absoluteTurningMotorEncoderChannel,
        double turningEncoderOffset,
        Translation2d wheelLocation,
        double wheelDiameter) {

    private static JSONParser parser = new JSONParser();

    public enum MotorLocation {
        FRONT_LEFT("front-left", Constants.Swerve.FRONT_LEFT_WHEEL_LOCATION),
        FRONT_RIGHT("front-right", Constants.Swerve.FRONT_RIGHT_WHEEL_LOCATION),
        BACK_LEFT("back-left", Constants.Swerve.BACK_LEFT_WHEEL_LOCATION),
        BACK_RIGHT("back-right", Constants.Swerve.BACK_RIGHT_WHEEL_LOCATION);

        private final String id;
        private final Translation2d wheelLocation;

        private MotorLocation(String id, Translation2d wheelLocation) {
            this.id = id;
            this.wheelLocation = wheelLocation;
        }

        private JSONObject getMotor(JSONObject obj) throws InvalidConfigException {
            return getProperty(obj, id, JSONObject.class);
        }
    }

    public static class InvalidConfigException extends Exception {
        public InvalidConfigException(String message) {
            super(message);
        }
    }

    /**
     * Loads module constants from deploy/swerve/motors.json file
     *
     * @param location Location of module
     */
    public static ModuleConstants fromConfig(MotorLocation location) throws InvalidConfigException {
        File configFile = new File(Filesystem.getDeployDirectory(), "swerve/motors.json");
        if (!configFile.exists()) throw new InvalidConfigException("Config file does not exist");

        JSONObject obj;
        try {
            FileReader reader = new FileReader(configFile, StandardCharsets.UTF_8);
            obj = (JSONObject) parser.parse(reader);
            reader.close();
        } catch (IOException | ParseException e) {
            ConsoleLogger.logError("Failed to read config file", e);
            throw new InvalidConfigException("Failed to read config file");
        }

        JSONObject motor = location.getMotor(obj);

        return new ModuleConstants(
                Math.toIntExact(getProperty(motor, "driveMotorChannel", Long.class)),
                Math.toIntExact(getProperty(motor, "turningMotorChannel", Long.class)),
                Math.toIntExact(getProperty(motor, "encoderChannel", Long.class)),
                getProperty(motor, "encoderOffset", Double.class),
                location.wheelLocation,
                getProperty(motor, "wheelDiameter", Double.class));
    }

    private static <T> T getProperty(JSONObject obj, String id, Class<T> type) throws InvalidConfigException {
        Object val = obj.get(id);
        if (val == null) throw new InvalidConfigException("No " + id + " property in config");
        if (!type.isInstance(val))
            throw new InvalidConfigException("Property " + id + " is not of type " + type.getSimpleName());
        return type.cast(val);
    }

    public static double getAverageWheelDiameter() {
        try {
            return (fromConfig(MotorLocation.FRONT_LEFT).wheelDiameter()
                            + fromConfig(MotorLocation.FRONT_RIGHT).wheelDiameter()
                            + fromConfig(MotorLocation.BACK_LEFT).wheelDiameter()
                            + fromConfig(MotorLocation.BACK_RIGHT).wheelDiameter())
                    / 4.0;
        } catch (InvalidConfigException e) {
            throw new IllegalStateException("Failed to load wheel diameter from config", e);
        }
    }
}
