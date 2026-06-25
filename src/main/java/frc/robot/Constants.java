// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ModuleConstants.DriveMotorType;
import frc.robot.utils.ModuleConstants.InvalidConfigException;
import frc.robot.utils.ModuleConstants.MotorLocation;
import frc.robot.utils.ModuleConstants.TurnMotorType;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.wrappers.ImmutableCurrent;
import frc.robot.utils.wrappers.ImmutableTime;
import frc.robot.utils.wrappers.Pose2d;
import frc.robot.utils.wrappers.Translation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;
import org.ironmaple.simulation.drivesims.COTS;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    private Constants() {}

    public final class Game {

        public static final AprilTagFieldLayout APRILTAG_LAYOUT =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        public static final int[] ALL_TAGS = Constants.Game.APRILTAG_LAYOUT.getTags().stream()
                .mapToInt(t -> t.ID)
                .toArray();

        public interface IGamePosition {
            Pose2d getPose();

            static <E extends IGamePosition> E closestTo(edu.wpi.first.math.geometry.Pose2d pose, E[] values) {
                E closest = null;
                double closestDistance = Double.MAX_VALUE;
                for (E pos : values) {
                    double distance = pos.getPose().getTranslation().getDistance(pose.getTranslation());
                    if (distance < closestDistance) {
                        closest = pos;
                        closestDistance = distance;
                    }
                }
                return closest;
            }

            static Pose2d[] aggregatePositions(IGamePosition[]... values) {
                List<Pose2d> poses = new ArrayList<>();
                for (IGamePosition[] value : values) {
                    for (IGamePosition pos : value) {
                        poses.add(pos.getPose());
                    }
                }
                return poses.toArray(new Pose2d[0]);
            }
        }

        public static final double HUB_RIM_HEIGHT_METERS = Units.feetToMeters(6);
        public static final double FUEL_DIAMETER_METERS = Units.inchesToMeters(5.906);

        public static final Translation3d BLUE_HUB_POSITION =
                new Translation3d(4.625594, 4.03463, Units.feetToMeters(6));
        public static final Translation3d RED_HUB_POSITION = new Translation3d(
                FlippingUtil.fieldSizeX - BLUE_HUB_POSITION.getX(),
                FlippingUtil.fieldSizeY - BLUE_HUB_POSITION.getY(),
                BLUE_HUB_POSITION.getZ());

        public static final Translation3d BLUE_PASSING_TARGET_HP_SIDE =
                new Translation3d(Units.inchesToMeters(40.0), 1.664, 0.0);
        public static final Translation3d BLUE_PASSING_TARGET_DEPOT_SIDE = new Translation3d(
                BLUE_PASSING_TARGET_HP_SIDE.getX(), FlippingUtil.fieldSizeY - BLUE_PASSING_TARGET_HP_SIDE.getY(), 0.0);

        public static final Translation3d RED_PASSING_TARGET_HP_SIDE = new Translation3d(
                FlippingUtil.fieldSizeX - BLUE_PASSING_TARGET_HP_SIDE.getX(),
                FlippingUtil.fieldSizeY - BLUE_PASSING_TARGET_HP_SIDE.getY(),
                0.0);
        public static final Translation3d RED_PASSING_TARGET_DEPOT_SIDE = new Translation3d(
                FlippingUtil.fieldSizeX - BLUE_PASSING_TARGET_DEPOT_SIDE.getX(),
                FlippingUtil.fieldSizeY - BLUE_PASSING_TARGET_DEPOT_SIDE.getY(),
                0.0);

        private Game() {}
    }

    public enum FieldStartingLocation {
        TrenchDepot(new Pose2d(4.408, 7.408, Rotation2d.kZero)),
        Center(new Pose2d(3.418, 4.080, Rotation2d.fromDegrees(34.439))),
        TrenchOutpost(new Pose2d(4.414, 0.630, Rotation2d.kZero));

        private final Pose2d transformRed;
        private final Pose2d transformBlue;

        private FieldStartingLocation(Pose2d poseBlue) {
            transformRed = Pose2d.toImmutable(FlippingUtil.flipFieldPose(poseBlue));
            transformBlue = poseBlue;
        }

        public Pose2d getPose(Alliance alliance) {
            return alliance == Alliance.Red ? transformRed : transformBlue;
        }

        public Pose2d getPose() {
            return getPose(DriverStationUtils.getCurrentAlliance());
        }
    }

    public static final class Align {

        public static final double MAX_VELOCITY = Constants.Swerve.MAX_MODULE_SPEED; // m/s
        public static final double MAX_ANGULAR_VELOCITY = 1.8; // rad/s
        public static final double MAX_ACCELERATION = 3.3228; // m/s^2
        public static final double MAX_DECELERATION = 11.0; // m/s^2
        public static final double MAX_JERK = 2 * 7.0711; // m/s^3
        public static final double MAX_DEJERK = 2 * 7.0711; // m/s^3

        public static final double TRANSLATIONAL_TOLERANCE = 0.01; // Meters
        public static final double TRANSLATIONAL_VELOCITY_TOLERANCE = 0.05; // Meters/s
        public static final double ROTATIONAL_TOLERANCE = Math.toRadians(1); // Radians
        public static final double ROTATIONAL_VELOCITY_TOLERANCE = Math.toRadians(5); // Radians/s

        public static final double ADDITIONAL_OFFSET = 0.02;

        private Align() {}
    }

    public static final class Vision {

        public static final String TURRET_NAME = "limelight-trt";
        public static final String LEFT_BACK_NAME = "left-back";
        public static final String RIGHT_FRONT_NAME = "limelight-fr";
        public static final String RIGHT_BACK_NAME = "right-back";
        public static final String INTAKE_LEFT_NAME = "intake-left";
        public static final String INTAKE_RIGHT_NAME = "intake-right";

        public static final Transform3d ROBOT_TO_CAMERA_LEFT_BACK = new Transform3d(
                new Translation3d(0.169875, 0.336431, 0.326706),
                new Rotation3d(
                        Units.degreesToRadians(-0.488769034869),
                        Units.degreesToRadians(30.7856074797),
                        Units.degreesToRadians(106.683871305)));
        public static final Transform3d ROBOT_TO_CAMERA_RIGHT_FRONT = new Transform3d(
                new Translation3d(0.188160, -0.298610, 0.471224),
                new Rotation3d(
                        Units.degreesToRadians(-0.00986101899264),
                        Units.degreesToRadians(29.9963964578),
                        Units.degreesToRadians(-48.8233556849)));
        public static final Transform3d ROBOT_TO_CAMERA_RIGHT_BACK = new Transform3d(
                new Translation3d(0.170356, -0.340075, 0.393292),
                new Rotation3d(
                        Units.degreesToRadians(-0.827613033167),
                        Units.degreesToRadians(25.5207428597),
                        Units.degreesToRadians(-126.219878553)));
        public static final Transform3d MECHANISM_TO_CAMERA_TURRET = new Transform3d(
                new Translation3d(0.183918, 0, 0.355131 + 0.192215),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(30), Units.degreesToRadians(0)));
        public static final Transform3d ROBOT_TO_CAMERA_INTAKE_LEFT = new Transform3d(
                new Translation3d(Meters.of(-0.338901), Meters.of(-0.277466), Meters.of(0.443145)),
                new Rotation3d(
                        Degrees.of(0),
                        Degrees.of(0),
                        Degrees.of(180.0 + 9.0))); // TODO: pitch down 25 deg, object detection sim breaks with pitch rn
        public static final Transform3d ROBOT_TO_CAMERA_INTAKE_RIGHT = new Transform3d(
                new Translation3d(Meters.of(-0.338901), Meters.of(0.277466), Meters.of(0.443145)),
                new Rotation3d(
                        Degrees.of(0),
                        Degrees.of(0),
                        Degrees.of(180.0 - 9.0))); // TODO: pitch down 25 deg, object detection sim breaks with pitch rn

        public enum VisionSimulationMode {
            /**
             * Uses PhotonVision simulation with accurate april tag placement on the field
             */
            PHOTON_SIM_ACCURATE(true),
            /**
             * Uses PhotonVision simulation with simulated inaccurate april tag placement on the field
             */
            PHOTON_SIM_INACCURATE(true),
            /**
             * Uses the maplesim actual robot pose as a vision measurement
             */
            MAPLE_CLEAN(false),
            /**
             * Uses the maplesim actual robot pose with noise added as a vision measurement
             */
            MAPLE_NOISE(false);

            final boolean isPhotonSim;

            VisionSimulationMode(boolean isPhotonSim) {
                this.isPhotonSim = isPhotonSim;
            }

            public boolean isPhotonSim() {
                return isPhotonSim;
            }
        }

        public enum ObjectDetectionSimulationMode {
            /**
             * Use external photonvision camera connected to the same network as simulation
             */
            PHOTONVISION,
            /**
             * Use maple sim with realistic object detection simulation
             * This includes noise, false positives, false negatives, field of view limitations, and other realistic factors.
             */
            MAPLE_SIM;
        }

        public enum PhotonVisionSimPerformanceMode {
            /**
             * Disables all streaming and unnecessary computer vision apart from AprilTag detection.
             * This is the fastest and should be used.
             */
            FAST,
            /**
             * Enables streaming of the camera feed to the dashboard.
             * This is not necessary for the simulation and can cause significant performance issues,
             * but can be useful for debugging robot to camera poses, etc.
             */
            STREAMED,
            /**
             * Same as STREAMED but also enables a field wireframe to help with the dashboard visualization.
             * This is the slowest and should only be used for debugging the field layout and robot to camera poses.
             */
            WIREFRAME
        }

        public static final VisionSimulationMode VISION_SIMULATION_MODE = VisionSimulationMode.PHOTON_SIM_ACCURATE;

        public static final PhotonVisionSimPerformanceMode PHOTON_VISION_SIM_PERFORMANCE_MODE =
                PhotonVisionSimPerformanceMode.FAST;

        public static final ObjectDetectionSimulationMode OBJECT_DETECTION_SIMULATION_MODE =
                ObjectDetectionSimulationMode.MAPLE_SIM;

        private Vision() {}
    }

    public static final class Assists {

        public static final Distance GROUND_ASSIST_MAX_DISTANCE = Meters.of(4);
        public static final Angle GROUND_ASSIST_MAX_ANGLE_ERROR = Degrees.of(60);
        public static final double GROUND_ASSIST_ROTATION_P = 5.0;
        public static final double GROUND_ASSIST_TRANSLATION_P = 5.0;

        private Assists() {}
    }

    public static final class Control {

        // Sensitivity for speed meter
        public static final double DIRECTIONAL_SPEED_METER_LOW = 0.25;
        public static final double SPIN_SPEED_METER_LOW = 0.5;

        // Sensitivies for directional controls (XY) and spin (theta)
        public static final double JOYSTICK_XY_SENSITIVITY = 1;
        public static final double JOYSTICK_SPIN_SENSITIVITY = 1;
        public static final double JOYSTICK_XY_THRESHOLD = 0.25;
        public static final double JOYSTICK_SPIN_THRESHOLD = 0.3;
        public static final double JOYSTICK_XY_EXPONENT = 2;
        public static final double JOYSTICK_SPIN_EXPONENT = 2;

        public static final double SPIN_KP = 4;
        public static final double SPIN_KD = 0.28;
        public static final Constraints SPIN_CONSTRAINTS = new Constraints(
                Constants.Swerve.MAX_ANGULAR_SPEED_RADIANS, Constants.Swerve.MAX_ANGULAR_ACCELERATION_RADIANS);
        public static final double JOYSTICK_ABSOLUTE_SPIN_THRESHOLD = 0.9;
        public static final double JOYSTICK_ABSOLUTE_SPIN_THRESHOLD_PACMAN = 0.2;

        public static final double RELATIVE_SPIN_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(140);

        private Control() {}
    }

    public static final class Frame {

        /**
         * Distance between wheels (width aka between left and right and length aka between front and
         * back). Used for calculating wheel locations on the robot
         */
        public static final double ROBOT_WHEEL_DISTANCE_WIDTH = 0.514350;

        public static final double ROBOT_WHEEL_DISTANCE_LENGTH = 0.514350;

        public static final double FRAME_WIDTH = Inches.of(27).in(Meters);
        public static final double FRAME_LENGTH = Inches.of(27).in(Meters);

        public static final double FRAME_WITH_BUMPER_WIDTH =
                FRAME_WIDTH + Inches.of(8.264).in(Meters);
        public static final double FRAME_WITH_BUMPER_LENGTH =
                FRAME_LENGTH + Inches.of(7.5).in(Meters);

        public static final double ROBOT_MASS_KG = 60.668; // kg
        public static final double ROBOT_MOI = 6.4482549519; // kg*m^2

        private Frame() {}
    }

    public static final class Swerve {

        public static final double PERIODIC = 0.02;

        // Works out module locations
        public static final double WHEEL_EXTENT_X = Frame.ROBOT_WHEEL_DISTANCE_WIDTH / 2;
        public static final double WHEEL_EXTENT_Y = Frame.ROBOT_WHEEL_DISTANCE_LENGTH / 2;
        // Distance from center of robot to wheel
        public static final double WHEEL_BASE_RADIUS = Math.hypot(WHEEL_EXTENT_X, WHEEL_EXTENT_Y);

        public static final Translation2d FRONT_LEFT_WHEEL_LOCATION = new Translation2d(WHEEL_EXTENT_X, WHEEL_EXTENT_Y);
        public static final Translation2d FRONT_RIGHT_WHEEL_LOCATION =
                new Translation2d(WHEEL_EXTENT_X, -WHEEL_EXTENT_Y);
        public static final Translation2d BACK_LEFT_WHEEL_LOCATION = new Translation2d(-WHEEL_EXTENT_X, WHEEL_EXTENT_Y);
        public static final Translation2d BACK_RIGHT_WHEEL_LOCATION =
                new Translation2d(-WHEEL_EXTENT_X, -WHEEL_EXTENT_Y);

        // Gear ratios for falcon and kraken
        public static final double FALCON_TURN_GEAR_RATIO =
                15.43; // (https://web.archive.org/web/20230117081053/https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options)
        public static final double FALCON_DRIVE_GEAR_RATIO =
                7.36; // (https://web.archive.org/web/20230117081053/https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options)

        public static final double KRAKEN_TURN_GEAR_RATIO = 13.3714;
        public static final double KRAKEN_DRIVE_GEAR_RATIO = 6.75; // X1 12 pinion

        public static final ImmutableCurrent FALCON_TURN_STATOR_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(120));
        public static final ImmutableCurrent FALCON_TURN_SUPPLY_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(70));
        public static final ImmutableCurrent FALCON_TURN_SUPPLY_LOWER_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(40));
        public static final ImmutableTime FALCON_TURN_SUPPLY_LOWER_CURRENT_LIMIT_TIME =
                ImmutableTime.of(Seconds.of(1.0));
        public static final ImmutableCurrent FALCON_DRIVE_STATOR_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(120));
        public static final ImmutableCurrent FALCON_DRIVE_SUPPLY_LOWER_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(40));
        public static final ImmutableTime FALCON_DRIVE_SUPPLY_LOWER_CURRENT_LIMIT_TIME =
                ImmutableTime.of(Seconds.of(1.0));
        public static final ImmutableCurrent FALCON_DRIVE_SUPPLY_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(70));

        public static final ImmutableCurrent KRAKEN_TURN_STATOR_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(40));
        public static final ImmutableCurrent KRAKEN_TURN_SUPPLY_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(25));
        public static final ImmutableCurrent KRAKEN_TURN_SUPPLY_LOWER_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(25));
        public static final ImmutableTime KRAKEN_TURN_SUPPLY_LOWER_CURRENT_LIMIT_TIME =
                ImmutableTime.of(Seconds.of(1.0));
        public static final ImmutableCurrent KRAKEN_DRIVE_STATOR_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(108));
        public static final ImmutableCurrent KRAKEN_DRIVE_SUPPLY_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(58));
        public static final ImmutableCurrent KRAKEN_DRIVE_SUPPLY_LOWER_CURRENT_LIMIT = ImmutableCurrent.of(Amps.of(40));
        public static final ImmutableTime KRAKEN_DRIVE_SUPPLY_LOWER_CURRENT_LIMIT_TIME =
                ImmutableTime.of(Seconds.of(1.0));

        // All of the DRIVE motor values treat the 4 drive motors as one gearbox, with one output. This means that
        // applying KRAKEN_DRIVE_KA to all drive motors will accelerate the whole robot at 1 m/s^2.
        /* Units:
         * Ks: V
         * Kv: V/(m/s)
         * Ka: V/(m/s^2)
         * Kp: V/(m/s)
         */
        public static final double FALCON_DRIVE_KS = 0.13192;
        public static final double FALCON_DRIVE_KV = 2.7547;
        public static final double FALCON_DRIVE_KA = 0.24758;
        public static final double FALCON_DRIVE_KP = 4.6957;

        public static final double FALCON_TURN_KV = 1.7214;
        public static final double FALCON_TURN_KA = 0.049311;
        public static final double FALCON_TURN_KS = 0.13609;
        public static final double FALCON_TURN_KP = 67.02;
        public static final double FALCON_TURN_KD = 3.2831;

        public static final double KRAKEN_DRIVE_KS = 0.16364;
        public static final double KRAKEN_DRIVE_KV = 2.4345;
        public static final double KRAKEN_DRIVE_KA = 0.23572;
        public static final double KRAKEN_DRIVE_KP = 3.8703;

        public static final double KRAKEN_TURN_KV = 1.3101;
        public static final double KRAKEN_TURN_KA = 0.063814;
        public static final double KRAKEN_TURN_KS = 0.62704;
        public static final double KRAKEN_TURN_KP = 68.174;
        public static final double KRAKEN_TURN_KD = 3.2858;

        public static final double FEEDFORWARD_KA = KRAKEN_DRIVE_KA * 1; // 4 is good

        // Wheel diameter
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4); // TODO: measure

        // Turn max velocity
        public static final double TURN_MAX_ANGULAR_VELOCITY = 5; // ROTATIONS / SECOND // TODO: measure

        public static final double MAX_ANGULAR_ACCELERATION_RADIANS =
                54.9727426818 / WHEEL_BASE_RADIUS; // do the math if you want but this is right trust

        public static final double DRIVE_MAX_ACCELERATION = 12.0 / KRAKEN_DRIVE_KA;
        public static final double DRIVE_MAX_JERK = DRIVE_MAX_ACCELERATION * 10;

        public static final double TURN_MMEXPO_KV = 1.5;
        public static final double TURN_MMEXPO_KA = 0.02;

        /** The max speed the robot can travel */
        public static final double MAX_MODULE_SPEED = 4.5; // TODO: measure

        // Offset rotation origin for testing turret feedforward
        public static final Translation2d TURRET_OFFSET = Translation2d.kZero; // new Translation2d(0.127, 0.127);

        public static final RobotConfig PP_DEFAULT_CONFIG = new RobotConfig(
                Constants.Frame.ROBOT_MASS_KG,
                Constants.Frame.ROBOT_MOI,
                new ModuleConfig(
                        WHEEL_DIAMETER / 2,
                        MAX_MODULE_SPEED,
                        COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                        DCMotor.getKrakenX60(1),
                        Constants.Swerve.KRAKEN_DRIVE_GEAR_RATIO,
                        Constants.Swerve.KRAKEN_DRIVE_SUPPLY_CURRENT_LIMIT.in(Amps),
                        1),
                FRONT_LEFT_WHEEL_LOCATION.minus(TURRET_OFFSET),
                FRONT_RIGHT_WHEEL_LOCATION.minus(TURRET_OFFSET),
                BACK_LEFT_WHEEL_LOCATION.minus(TURRET_OFFSET),
                BACK_RIGHT_WHEEL_LOCATION.minus(TURRET_OFFSET));

        public static final PPHolonomicDriveController PP_DRIVE_CONTROLLER = new PPHolonomicDriveController(
                new PIDConstants(3.0, 0.0, 0.15), // Translation PID constants
                new PIDConstants(4.0, 0.0, 0.3) // Rotation PID constants
                );

        public static final DriveMotorType driveMotorType = DriveMotorType.KRAKEN;
        public static final TurnMotorType turnMotorType = TurnMotorType.KRAKEN;

        public static final Double MAX_ANGULAR_SPEED_RADIANS =
                Constants.Swerve.MAX_MODULE_SPEED / Constants.Swerve.WHEEL_BASE_RADIUS; // trust

        private Swerve() {}

        // Module Creation

        public static ModuleConstants getFrontLeftConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.FRONT_LEFT, driveMotorType, turnMotorType);
        }

        public static ModuleConstants getFrontRightConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.FRONT_RIGHT, driveMotorType, turnMotorType);
        }

        public static ModuleConstants getBackLeftConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.BACK_LEFT, driveMotorType, turnMotorType);
        }

        public static ModuleConstants getBackRightConstants() throws InvalidConfigException {
            return ModuleConstants.fromConfig(MotorLocation.BACK_RIGHT, driveMotorType, turnMotorType);
        }
    }

    public static final class Auto {

        public static final Time SOURCE_TIMEOUT = Seconds.of(0.8);

        private Auto() {}
    }

    public static final class Intake {

        // TODO
        public static final double EJECT_FUEL_PER_SECOND = 10.0;

        public static final Current ARM_SUPPLY_CURRENT_LIMIT = Amps.of(60);
        public static final Current ARM_SUPPLY_LOWER_CURRENT_LIMIT = Amps.of(20);
        public static final Time ARM_SUPPLY_LOWER_CURRENT_LIMIT_TIME = Seconds.of(1.0);
        public static final Current ARM_STATOR_CURRENT_LIMIT = Amps.of(100);

        public static final Current WHEEL_SUPPLY_CURRENT_LIMIT = Amps.of(20);
        public static final Current WHEEL_SUPPLY_LOWER_CURRENT_LIMIT = Amps.of(20);
        public static final Time WHEEL_SUPPLY_LOWER_CURRENT_LIMIT_TIME = Seconds.of(1.0);
        public static final Current WHEEL_STATOR_CURRENT_LIMIT = Amps.of(80);

        public static final double ARM_MMEXPO_KV = 4;
        public static final double ARM_MMEXPO_KA = 2;

        public static final double ARM_KP = 20;
        public static final double ARM_KD = 3;
        public static final double ARM_KG = 0.9;
        public static final double ARM_KS = 0.2509765625;
        public static final double ARM_KV = 0.8;
        public static final double ARM_KA = 0.3;

        public static final double WHEEL_KP = 3.1649;
        public static final double WHEEL_KS = 0.28151;
        public static final double WHEEL_KV = 2.3535;
        public static final double WHEEL_KA = 0.059602;

        public static final double WHEEL_MAX_ACCELERATION = 190;
        public static final double WHEEL_MAX_JERK = 590;

        public static final double ARM_DOWN_POSITION_RADIANS = Units.degreesToRadians(0.0); // should always be 0
        public static final double ARM_RETRACTED_POSITION_RADIANS = Units.degreesToRadians(81.0);
        public static final double ARM_MAX_POSITION_RADIANS = Units.degreesToRadians(137.0);
        public static final double ARM_STARTING_POSITION_RADIANS =
                SysIdManager.getProvider() instanceof frc.robot.subsystems.Intake.SysIdArm
                        ? ARM_DOWN_POSITION_RADIANS
                        : ARM_MAX_POSITION_RADIANS;
        public static final double ARM_UP_POSITION_RADIANS = ARM_MAX_POSITION_RADIANS;
        public static final double ARM_EJECT_POSITION_RADIANS =
                Units.degreesToRadians(13.4); // A bit up to stop balls wedging against the ground
        public static final double ARM_DOWN_FF = -1.0;
        public static final double ARM_UP_FF = 10;
        public static final double ARM_UP_HOLD_FF = 6.7;

        public static final double WHEEL_INTAKE_VELOCITY_MPS = 4.76036363636;
        public static final double WHEEL_JAMMED_VELOCITY_MPS = 4.76036363636;
        public static final double WHEEL_EJECT_VELOCITY_MPS = -4.76036363636;

        public static final double ARM_GEAR_RATIO = 12.8571428571;
        public static final double ARM_GRAVITY_POSITION_OFFSET_ROTATIONS = -0.082671;

        public static final double WHEEL_GEAR_RATIO = 3.05555555556;

        public static final Distance ROLLER_DIAMETER = Inches.of(1.875);

        public static final double WHEEL_METERS_PER_ROTATION = ROLLER_DIAMETER.in(Meter) * Math.PI / WHEEL_GEAR_RATIO;

        private Intake() {}
    }

    public static final class Turret {
        public static final double KP = 50;
        public static final double KD = 3.811;
        public static final double KS = 0.3;
        public static final double KV = 2.1;
        public static final double KA_MM = 0.01;
        public static final double KA = 0.25;
        public static final double KVP = 0.2;
        public static double FF_MUL = 0.72;
        public static double LOOKAHEAD_TIME = 0.1;

        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);
        public static final Current SUPPLY_LOWER_CURRENT_LIMIT = Amps.of(40);
        public static final Time SUPPLY_LOWER_CURRENT_LIMIT_TIME = Seconds.of(1.0);
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(90);

        public static final double MMEXPO_KV = 1.931;
        public static final double MMEXPO_KA = 1.1;

        public static final double GEAR_RATIO = 15.5428571429;

        public static final double ROTATION_LIMIT_INSET_ROTATIONS = 0.002;
        public static final double ROTATION_MAX_POSITION_MOTOR_ROTATIONS = 0.7126 - ROTATION_LIMIT_INSET_ROTATIONS;
        public static final double ROTATION_MIN_POSITION_MOTOR_ROTATIONS = -0.614 + ROTATION_LIMIT_INSET_ROTATIONS;

        public static final double FRONT_LEFT_MAGNET_MOTOR_ROTATIONS_CW = -0.044678;
        public static final double FRONT_LEFT_MAGNET_MOTOR_ROTATIONS_CCW = -0.060547;

        public static final double BACK_LEFT_MAGNET_MOTOR_ROTATIONS_CW = 0.206299;
        public static final double BACK_LEFT_MAGNET_MOTOR_ROTATIONS_CCW = 0.191406;

        public static final double BACK_RIGHT_MAGNET_MOTOR_ROTATIONS_CW = 0.453125;
        public static final double BACK_RIGHT_MAGNET_MOTOR_ROTATIONS_CCW = 0.437012;

        public static final double TURRET_SPRING_HIGH_START_POS = 0.343;
        public static final double TURRET_SPRING_HIGH_START_NEG = -0.297;
        public static final double TURRET_SPRING_LOW_START_POS = 0.343;
        public static final double TURRET_SPRING_LOW_START_NEG = -0.297;
        public static double TURRET_SPRING_HIGH_VOLTS = 1.11;
        public static double TURRET_SPRING_LOW_VOLTS = 1.11;

        public static final double STARTING_POSITION_RADIANS = Units.degreesToRadians(90.0);

        public static final double MAGNETIC_LIMIT_SWITCH_TRIGGER_ANGLE_RAD = Units.degreesToRadians(5);
        public static final double MAGNETIC_LIMIT_SWITCH_DETRIGGER_ANGLE_RAD = Units.degreesToRadians(10);
        public static final double TURRET_MAGNET_OFFSET_ANGLE_RAD = Units.degreesToRadians(27.7335249689);

        public static final double FRONT_LEFT_LIMIT_SWITCH_POSITION_RADIANS = Units.degreesToRadians(45.0);
        public static final double BACK_LEFT_LIMIT_SWITCH_POSITION_RADIANS = Units.degreesToRadians(135.0);
        public static final double BACK_RIGHT_LIMIT_SWITCH_POSITION_RADIANS = Units.degreesToRadians(225.0);

        private Turret() {}
    }

    public static final class Shooter {
        public static final double HOOD_KP = 31.295;
        public static final double HOOD_KD = 2.1075;
        public static final double HOOD_KS = 0.12011;
        public static final double HOOD_KV = 2.7761;
        public static final double HOOD_KA = 0.013839;
        public static final double HOOD_KG = 0.084491;
        public static final double HOOD_GRAVITY_POSITION_OFFSET_ROTATIONS = 0.4676;

        public static final double FLYWHEEL_KP = 0.04;
        public static final double FLYWHEEL_KS = 0.10324;
        public static final double FLYWHEEL_KV = 0.35945;
        public static final double FLYWHEEL_KA = 0.01;

        public static final Current HOOD_SUPPLY_CURRENT_LIMIT = Amps.of(20);
        public static final Current HOOD_SUPPLY_LOWER_CURRENT_LIMIT = Amps.of(10);
        public static final Time HOOD_SUPPLY_LOWER_CURRENT_LIMIT_TIME = Seconds.of(1.0);
        public static final Current HOOD_STATOR_CURRENT_LIMIT = Amps.of(40);

        public static final Current FLYWHEEL_SUPPLY_CURRENT_LIMIT = Amps.of(70);
        public static final Current FLYWHEEL_SUPPLY_LOWER_CURRENT_LIMIT = Amps.of(40);
        public static final Time FLYWHEEL_SUPPLY_LOWER_CURRENT_LIMIT_TIME = Seconds.of(1.0);
        public static final Current FLYWHEEL_STATOR_CURRENT_LIMIT = Amps.of(100);

        public static final double HOOD_MMEXPO_KV = HOOD_KV;
        public static final double HOOD_MMEXPO_KA = 0.3;

        public static final double FLYWHEEL_MAX_ACCELERATION = 500;
        public static final double FLYWHEEL_MAX_JERK = 0;

        public static final double HOOD_GEAR_RATIO = 35.0476190476;

        public static final double FLYWHEEL_GEAR_RATIO = 1;

        public static final Distance FLYWHEEL_WHEEL_DIAMETER = Inches.of(4);

        public static final double FLYWHEEL_METERS_PER_ROTATION =
                FLYWHEEL_WHEEL_DIAMETER.in(Meter) * Math.PI / FLYWHEEL_GEAR_RATIO;

        public static final double HOOD_STARTING_POSITION_RADIANS = Units.degreesToRadians(68.906250);
        public static final double HOOD_MAX_POSITION_RADIANS = HOOD_STARTING_POSITION_RADIANS;
        public static final double HOOD_MIN_POSITION_RADIANS = Units.degreesToRadians(35.871753);
        public static final double HOOD_FUEL_EXIT_ANGLE_OFFSET_RADIANS = Units.degreesToRadians(4);

        private Shooter() {}
    }

    public static final class Spindexer {
        public static final double KP = 0.0023175;
        public static final double KS = 0.073451;
        public static final double KV = 0.23114;
        public static final double KA = 0.011727;

        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);
        public static final Current SUPPLY_LOWER_CURRENT_LIMIT = Amps.of(40);
        public static final Time SUPPLY_LOWER_CURRENT_LIMIT_TIME = Seconds.of(1.0);
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(80);

        public static final double MAX_ACCELERATION = 200;
        public static final double MAX_JERK = 3600;

        public static final double GEAR_RATIO = 2.0;

        public static final double INTAKE_VELOCITY_RPS = 21.0;
        public static final double UNSTUCK_VELOCITY_RPS = -5.0;

        private Spindexer() {}
    }

    public static final class Feeder {
        public static final double KP = 0.0062925;
        public static final double KS = 0.12382;
        public static final double KV = 0.11573;
        public static final double KA = 0.0026933;

        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(35);
        public static final Current SUPPLY_LOWER_CURRENT_LIMIT = Amps.of(40);
        public static final Time SUPPLY_LOWER_CURRENT_LIMIT_TIME = Seconds.of(1.0);
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(80);

        public static final double MAX_ACCELERATION = 700;
        public static final double MAX_JERK = 6600;

        public static final double GEAR_RATIO = 1.0;

        public static final double INTAKE_VELOCITY_RPS = 50.0;

        public static final Distance WHEEL_DIAMETER = Inches.of(2.0);

        private Feeder() {}
    }

    public enum ClimberHeight {
        DOWN(0),
        UP(Units.inchesToMeters(8.0));

        private static final double HEIGHT_DIFFERENCE_TOLERANCE = 0.1; // meters

        private final double height;

        private ClimberHeight(double heightMeters) {
            this.height = heightMeters;
        }

        public double getHeight() {
            return height;
        }

        public double getDifference(double height) {
            return Math.abs(this.height - height) / HEIGHT_DIFFERENCE_TOLERANCE; // TODO what is this division doing???
        }
    }

    public static final class Climber {
        public static final double KV = 46.657;
        public static final double KA = 0.88771;
        public static final double KG = 0.033723;
        public static final double KS = 0.077735;

        public static final double KP = 47.973;
        public static final double KD = 34.1;

        public static final double MMEXPO_KV = KV;
        public static final double MMEXPO_KA = 10;

        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(70);
        public static final Current SUPPLY_CURRENT_LOWER_LIMIT = Amps.of(40);
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(120);

        public static final double GEAR_RATIO = 48;
        public static final double SPROCKET_EFFECTIVE_RADIUS = Units.inchesToMeters(
                0.75); // MUST BE EFFECTIVE RADIUS, NOT PHYSICAL RADIUS. This is the radius at which it contacts the
        // rack, which is not the same as the physical radius of the sprocket due to the sprocket
        // entering into the rack.
        public static final double METERS_PER_ROTATION = SPROCKET_EFFECTIVE_RADIUS
                * 2
                * Math.PI
                / GEAR_RATIO; // 2 * pi * r / gear ratio because same as getting distance a wheelmoved, just vertically

        public static final double AT_GOAL_POSITION_TOLERANCE = 0.03; // TODO make correct
        public static final double AT_GOAL_VELOCITY_TOLERANCE = 0.63514; // TODO make correct

        public static final double MAX_HEIGHT_METERS = Units.inchesToMeters(8.0);

        public static final double CARRIAGE_MASS_KG = 0.706915816;

        public static final List<Translation3d> END_OF_TOWER_POSITIONS = List.of(
                // always in the center of the round tower rung, NOT THE TOP OF THE RUNG
                // blue
                SimpleMath.fromCenterFieldRelativeTranslation3d(new Translation3d(7.208488, 0.8112125, 0.685800)),
                SimpleMath.fromCenterFieldRelativeTranslation3d(new Translation3d(7.208488, -0.2333625, 0.685800)),
                // red
                SimpleMath.fromCenterFieldRelativeTranslation3d(new Translation3d(-7.208488, -0.8112125, 0.685800)),
                SimpleMath.fromCenterFieldRelativeTranslation3d(new Translation3d(-7.208488, 0.2333625, 0.685800)));

        public static final double END_OF_TOWER_POSITION_TOLERANCE = Units.inchesToMeters(
                        2.9375) // half of the width of the part of the rung that sticks out
                + Units.inchesToMeters(1.0); // a little bit extra because sim doesn't have bumper gap yet TODO remove

        public static final Predicate<Translation3d> AT_END_OF_TOWER_POSITION_PREDICATE = pos -> {
            for (Translation3d endPos : END_OF_TOWER_POSITIONS) {
                if (pos.getDistance(endPos) < END_OF_TOWER_POSITION_TOLERANCE) {
                    return true;
                }
            }
            return false;
        };

        public static final Transform2d ROBOT_TO_CLIMBER_OFFSET = new Transform2d(
                0.292100,
                -0.119634,
                new Rotation2d()); // TODO this is centered on climber, should be centered on hook, make correct
        public static final double CLIMBER_BASE_HEIGHT_METERS = 0.502432;

        public static final double CLIMBER_SAFE_SHOTBLOCKER_HEIGHT = Units.inchesToMeters(6.0);
        public static final double CLIMBER_SHOTBLOCKER_EXTEND_START_HEIGHT = Units.inchesToMeters(6.3);
        public static final double CLIMBER_SHOTBLOCKER_EXTEND_END_HEIGHT = Units.inchesToMeters(7.5);

        private Climber() {}
    }

    public final class RobotState {

        public static final boolean MOTOR_LOGGING_ENABLED = false;

        public static final AutoLogLevel.Level AUTO_LOG_LEVEL = getAutoLogLevel();

        /**
         * <p>
         * Enable NT and Advantage Scope for unit tests.
         * </p>
         * <p>
         * WARNING! ONLY ENABLE THIS IF RUNNING A SINGLE UNIT TEST
         * </p>
         * <p>
         * RUNNING TESTS IN PARALLEL IS NOT SUPPORTED
         * </p>
         *
         * Example for running just one test: {@code ./gradlew test --tests "INSERT_TEST_NAME_HERE"}
         */
        public static final boolean UNIT_TESTS_ENABLE_ADVANTAGE_SCOPE = false;

        private static boolean runningAsUnitTest = false;

        private RobotState() {}

        public static void setRunningAsUnitTest() {
            runningAsUnitTest = true;
        }

        public static Mode getMode() {
            if (RobotBase.isReal()) return Mode.REAL;
            if (runningAsUnitTest) return Mode.TEST;
            return Mode.SIM; // change to REPLAY when replaying
        }

        private static AutoLogLevel.Level getAutoLogLevel() {
            if (RobotBase.isReal()) {
                return SysIdManager.getProvider().isEnabled() ? AutoLogLevel.Level.SYSID : AutoLogLevel.Level.REAL;
            } else {
                return AutoLogLevel.Level.SIM;
            }
        }

        public enum Mode {
            REAL(true),
            SIM(true),
            REPLAY(false),
            TEST(false);

            final boolean realtime;

            Mode(boolean realtime) {
                this.realtime = realtime;
            }

            public boolean isRealtime() {
                return realtime;
            }
        }
    }
}
