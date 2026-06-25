package frc.robot.utils;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.utils.wrappers.ImmutableCurrent;
import frc.robot.utils.wrappers.ImmutableTime;
import frc.robot.utils.wrappers.Translation2d;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

/**
 * essentially serves as a storage unit for one swerve module, storing every single constant that
 * a module might want to use
 *
 * @param driveMotorChannel drive motor port
 * @param turningMotorChannel turn motor port
 * @param absoluteTurningMotorEncoderChannel abs turn motor encoder port
 * @param turningEncoderOffset offset of the abs turn encoder at a set starting position (which we
 *     found through manually testing)
 * @param wheelLocation Translation2d object of where the wheel is relative to robot frame
 * @param turnGearRatio Gear ratio of the turn motor
 * @param driveGearRatio Gear ratio of the drive motor
 * @param turnMotorSupplyCurrentLimit Supply current limit of the turn motor
 * @param turnMotorStatorCurrentLimit Stator current limit of the turn motor
 * @param driveMotorSupplyCurrentLimit Supply current limit of the drive motor
 * @param driveMotorStatorCurrentLimit Stator current limit of the drive motor
 * @param driveKv Drive motor velocity constant
 * @param driveKa Drive motor acceleration constant
 * @param driveKs Drive motor static constant
 * @param driveKp Drive motor proportional gain
 * @param turnKv Turn motor velocity constant
 * @param turnKa Turn motor acceleration constant
 * @param turnKs Turn motor static constant
 * @param turnKp Turn motor proportional gain
 * @param turnKd Turn motor derivative gain
 * @param wheelDiameter Diameter of the wheel
 */
public record ModuleConstants(
        int driveMotorChannel,
        int turningMotorChannel,
        int absoluteTurningMotorEncoderChannel,
        double turningEncoderOffset,
        Translation2d wheelLocation,
        double turnGearRatio,
        double driveGearRatio,
        Current turnMotorSupplyCurrentLimit,
        Current turnMotorSupplyLowerCurrentLimit,
        Time turnMotorSupplyLowerCurrentLimitTime,
        Current turnMotorStatorCurrentLimit,
        Current driveMotorSupplyCurrentLimit,
        Current driveMotorSupplyLowerCurrentLimit,
        Time driveMotorSupplyLowerCurrentLimitTime,
        Current driveMotorStatorCurrentLimit,
        double driveKv,
        double driveKa,
        double driveKs,
        double driveKp,
        double turnKv,
        double turnKa,
        double turnKs,
        double turnKp,
        double turnKd,
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

    private interface MotorType {
        String getJsonId();

        default double getEncoderOffset(JSONObject encoderOffset) throws InvalidConfigException {
            return ModuleConstants.getProperty(encoderOffset, getJsonId(), Double.class);
        }
    }

    public enum TurnMotorType implements MotorType {
        FALCON(
                "falcon",
                Constants.Swerve.FALCON_TURN_GEAR_RATIO,
                Constants.Swerve.FALCON_TURN_SUPPLY_CURRENT_LIMIT,
                Constants.Swerve.FALCON_TURN_SUPPLY_LOWER_CURRENT_LIMIT,
                Constants.Swerve.FALCON_TURN_SUPPLY_LOWER_CURRENT_LIMIT_TIME,
                Constants.Swerve.FALCON_TURN_STATOR_CURRENT_LIMIT,
                Constants.Swerve.FALCON_TURN_KV,
                Constants.Swerve.FALCON_TURN_KA,
                Constants.Swerve.FALCON_TURN_KS,
                Constants.Swerve.FALCON_TURN_KP,
                Constants.Swerve.FALCON_TURN_KD),
        KRAKEN(
                "kraken",
                Constants.Swerve.KRAKEN_TURN_GEAR_RATIO,
                Constants.Swerve.KRAKEN_TURN_SUPPLY_CURRENT_LIMIT,
                Constants.Swerve.KRAKEN_TURN_SUPPLY_LOWER_CURRENT_LIMIT,
                Constants.Swerve.KRAKEN_TURN_SUPPLY_LOWER_CURRENT_LIMIT_TIME,
                Constants.Swerve.KRAKEN_TURN_STATOR_CURRENT_LIMIT,
                Constants.Swerve.KRAKEN_TURN_KV,
                Constants.Swerve.KRAKEN_TURN_KA,
                Constants.Swerve.KRAKEN_TURN_KS,
                Constants.Swerve.KRAKEN_TURN_KP,
                Constants.Swerve.KRAKEN_TURN_KD);

        private final String jsonId;
        private final double gearRatio;
        private final ImmutableCurrent supplyCurrentLimit;
        private final ImmutableCurrent supplyLowerCurrentLimit;
        private final ImmutableTime supplyLowerCurrentLimitTime;
        private final ImmutableCurrent statorCurrentLimit;

        private final double kV;
        private final double kA;
        private final double kS;
        private final double kP;
        private final double kD;

        TurnMotorType(
                String jsonId,
                double gearRatio,
                ImmutableCurrent supplyCurrentLimit,
                ImmutableCurrent supplyLowerCurrentLimit,
                ImmutableTime supplyLowerCurrentLimitTime,
                ImmutableCurrent statorCurrentLimit,
                double kV,
                double kA,
                double kS,
                double kP,
                double kD) {
            this.jsonId = jsonId;
            this.gearRatio = gearRatio;
            this.supplyCurrentLimit = supplyCurrentLimit;
            this.supplyLowerCurrentLimit = supplyLowerCurrentLimit;
            this.supplyLowerCurrentLimitTime = supplyLowerCurrentLimitTime;
            this.statorCurrentLimit = statorCurrentLimit;
            this.kV = kV;
            this.kA = kA;
            this.kS = kS;
            this.kP = kP;
            this.kD = kD;
        }

        @Override
        public String getJsonId() {
            return jsonId;
        }
    }

    public enum DriveMotorType implements MotorType {
        FALCON(
                "falcon",
                Constants.Swerve.FALCON_DRIVE_GEAR_RATIO,
                Constants.Swerve.FALCON_DRIVE_SUPPLY_CURRENT_LIMIT,
                Constants.Swerve.FALCON_DRIVE_SUPPLY_LOWER_CURRENT_LIMIT,
                Constants.Swerve.FALCON_DRIVE_SUPPLY_LOWER_CURRENT_LIMIT_TIME,
                Constants.Swerve.FALCON_DRIVE_STATOR_CURRENT_LIMIT,
                Constants.Swerve.FALCON_DRIVE_KV,
                Constants.Swerve.FALCON_DRIVE_KA,
                Constants.Swerve.FALCON_DRIVE_KS,
                Constants.Swerve.FALCON_DRIVE_KP),
        KRAKEN(
                "kraken",
                Constants.Swerve.KRAKEN_DRIVE_GEAR_RATIO,
                Constants.Swerve.KRAKEN_DRIVE_SUPPLY_CURRENT_LIMIT,
                Constants.Swerve.KRAKEN_DRIVE_SUPPLY_LOWER_CURRENT_LIMIT,
                Constants.Swerve.KRAKEN_DRIVE_SUPPLY_LOWER_CURRENT_LIMIT_TIME,
                Constants.Swerve.KRAKEN_DRIVE_STATOR_CURRENT_LIMIT,
                Constants.Swerve.KRAKEN_DRIVE_KV,
                Constants.Swerve.KRAKEN_DRIVE_KA,
                Constants.Swerve.KRAKEN_DRIVE_KS,
                Constants.Swerve.KRAKEN_DRIVE_KP);

        private final String jsonId;
        private final double gearRatio;
        private final ImmutableCurrent supplyCurrentLimit;
        private final ImmutableCurrent supplyLowerCurrentLimit;
        private final ImmutableTime supplyLowerCurrentLimitTime;
        private final ImmutableCurrent statorCurrentLimit;

        private final double kV;
        private final double kA;
        private final double kS;
        private final double kP;

        DriveMotorType(
                String jsonId,
                double gearRatio,
                ImmutableCurrent supplyCurrentLimit,
                ImmutableCurrent supplyLowerCurrentLimit,
                ImmutableTime supplyLowerCurrentLimitTime,
                ImmutableCurrent statorCurrentLimit,
                double kV,
                double kA,
                double kS,
                double kP) {
            this.jsonId = jsonId;
            this.gearRatio = gearRatio;
            this.supplyCurrentLimit = supplyCurrentLimit;
            this.statorCurrentLimit = statorCurrentLimit;
            this.supplyLowerCurrentLimit = supplyLowerCurrentLimit;
            this.supplyLowerCurrentLimitTime = supplyLowerCurrentLimitTime;
            this.kV = kV;
            this.kA = kA;
            this.kS = kS;
            this.kP = kP;
        }

        @Override
        public String getJsonId() {
            return jsonId;
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
     * @param driveMotorType Drive motor type (also determines the falcon/kraken objects in config)
     * @param turnMotorType Turn motor type (only used for the final module creation)
     */
    public static ModuleConstants fromConfig(
            MotorLocation location, DriveMotorType driveMotorType, TurnMotorType turnMotorType)
            throws InvalidConfigException {
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
        JSONObject encoderOffset = getProperty(motor, "encoderOffset", JSONObject.class);

        return new ModuleConstants(
                Math.toIntExact(getProperty(motor, "driveMotorChannel", Long.class)),
                Math.toIntExact(getProperty(motor, "turningMotorChannel", Long.class)),
                Math.toIntExact(getProperty(motor, "encoderChannel", Long.class)),
                driveMotorType.getEncoderOffset(encoderOffset),
                location.wheelLocation,
                turnMotorType.gearRatio,
                driveMotorType.gearRatio,
                turnMotorType.supplyCurrentLimit,
                turnMotorType.supplyLowerCurrentLimit,
                turnMotorType.supplyLowerCurrentLimitTime,
                turnMotorType.statorCurrentLimit,
                driveMotorType.supplyCurrentLimit,
                driveMotorType.supplyLowerCurrentLimit,
                driveMotorType.supplyLowerCurrentLimitTime,
                driveMotorType.statorCurrentLimit,
                driveMotorType.kV,
                driveMotorType.kA,
                driveMotorType.kS,
                driveMotorType.kP,
                turnMotorType.kV,
                turnMotorType.kA,
                turnMotorType.kS,
                turnMotorType.kP,
                turnMotorType.kD,
                Constants.Swerve.WHEEL_DIAMETER);
    }

    private static <T> T getProperty(JSONObject obj, String id, Class<T> type) throws InvalidConfigException {
        Object val = obj.get(id);
        if (val == null) throw new InvalidConfigException("No " + id + " property in config");
        if (!type.isInstance(val))
            throw new InvalidConfigException("Property " + id + " is not of type " + type.getSimpleName());
        return type.cast(val);
    }
}
