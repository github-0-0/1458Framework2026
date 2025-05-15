package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.RobotConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.control.PIDFConstants;
import frc.robot.lib.swerve.COTSTalonFXSwerveConstants;
import frc.robot.lib.swerve.SwerveModuleConstants;
import frc.robot.subsystems.Drive.KinematicLimits;
import frc.robot.subsystems.vision.VisionDeviceConstants;

public final class Constants {
    public static final double STICK_DEADBAND = 0.07;
    public static boolean isEpsilon;

    // robot loop time
	public static final double LOOPER_DT = 0.02; // 50 fps
    // Disables extra smart dashboard outputs that slow down the robot
	public static final boolean DISABLE_EXTRA_TELEMETRY = false;

    /* Define subsystem constants below */
    public static final class Swerve {
        public static final int PIGEON_ID = 20;

        public static final COTSTalonFXSwerveConstants CHOSEN_MODULE =  
            COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(24); 
        public static final double WHEEL_BASE = Units.inchesToMeters(24);
        public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;
        public static final double WHEEL_DIAMETER = Swerve.CHOSEN_MODULE.wheelDiameter; 

        /* Swerve Kinematics */
        public static final Translation2d[] MODULE_LOCATIONS = {
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        };
        
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            MODULE_LOCATIONS[0],
            MODULE_LOCATIONS[1],
            MODULE_LOCATIONS[2],
            MODULE_LOCATIONS[3]
        );

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGEL_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final InvertedValue DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue CANCODER_INVERT = CHOSEN_MODULE.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 20;
        public static final int ANGLE_CURRENT_THRESHOLD = 30;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CURRENT_LIMIT = 30;
        public static final int DRIVE_CURRENT_THRESHOLD = 45; 
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = 15.0;
        public static final double ANGLE_KI = CHOSEN_MODULE.angleKI;
        public static final double ANGLE_KD = CHOSEN_MODULE.angleKD;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.12;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0004;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = 0.32;
        public static final double DRIVE_KV = 1.51;
        public static final double DRIVE_KA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4;
        public static final double MAX_AUTO_SPEED = MAX_SPEED * 0.85;  // Max out at 85% to ensure attainable speeds, 

        /** Meters per Second Squared */
        public static final double MAX_ACCELERATION = 3;
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = Math.PI * 3; //TODO: This must be tuned to specific robot

        /** Radians per Second Squared */
        public static final double MAX_ANGULAR_ACCELERATION = 720.0;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;


        public static final boolean INVERT_GYRO = false; // TODO: ensure Gyro is CCW+ CW-

        /* Heading Controller */

        // Stabilize Heading PID Values
        public static final PIDFConstants STABILIZE_HEADING_PIDF_CONSTANTS = new PIDFConstants(
            10.0, 0.0, 0.3, 2.0);

        // Stabilize Heading PID Values
        public static final PIDFConstants SNAP_HEADING_PIDF_CONSTANTS = new PIDFConstants(
            10.0, 0.0, 0.6, 1.0);

        // Motor PIDs
        public static final double DRIVE_MOTOR_KV = 12 * Math.PI * WHEEL_DIAMETER / (DRIVE_GEAR_RATIO * MAX_SPEED);

        public static final PIDFConstants ANGLE_MOTOR_PIDF_CONSTANTS = new PIDFConstants(
            10.0, 0.0, 0.1, 1.0);
        public static final PIDFConstants DRIVE_MOTOR_PIDF_CONSTANTS = new PIDFConstants(
            1.2, 0.005, 0.0, DRIVE_MOTOR_KV);

        public static final double CANCODER_BOOT_ALLOWANCE_SECONDS = 10.0;

        public static final KinematicLimits UNCAPPED_KINEMATIC_LIMITS = new KinematicLimits();

        static {
            UNCAPPED_KINEMATIC_LIMITS.kMaxDriveVelocity = MAX_SPEED;
            UNCAPPED_KINEMATIC_LIMITS.kMaxAccel = Double.MAX_VALUE;
            UNCAPPED_KINEMATIC_LIMITS.kMaxAngularVelocity = Swerve.MAX_ANGULAR_VELOCITY;
            UNCAPPED_KINEMATIC_LIMITS.kMaxAngularAccel = Double.MAX_VALUE;
        }

        /* Swerve module configurations */
        public static TalonFXConfiguration AngleMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.Slot0.kP = ANGLE_MOTOR_PIDF_CONSTANTS.kP;
            config.Slot0.kI = ANGLE_MOTOR_PIDF_CONSTANTS.kI;
            config.Slot0.kD = ANGLE_MOTOR_PIDF_CONSTANTS.kD;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = ANGLE_MOTOR_PIDF_CONSTANTS.kF;

            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = Swerve.ANGLE_CURRENT_LIMIT;

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = Swerve.ANGLE_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLowerLimit = Swerve.ANGLE_CURRENT_THRESHOLD;
            config.CurrentLimits.SupplyCurrentLowerTime = Swerve.ANGLE_CURRENT_THRESHOLD_TIME;

            config.Voltage.PeakForwardVoltage = 12.0;
            config.Voltage.PeakReverseVoltage = -12.0;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            return config;
        }

        public static TalonFXConfiguration DriveMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.Slot0.kP = DRIVE_MOTOR_PIDF_CONSTANTS.kP;
            config.Slot0.kI = DRIVE_MOTOR_PIDF_CONSTANTS.kI;
            config.Slot0.kD = DRIVE_MOTOR_PIDF_CONSTANTS.kD;
            config.Slot0.kS = 0.1;
            config.Slot0.kV = DRIVE_MOTOR_PIDF_CONSTANTS.kF;

            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = Swerve.DRIVE_CURRENT_LIMIT;

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = Swerve.DRIVE_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLowerLimit = Swerve.DRIVE_CURRENT_THRESHOLD;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.5;

            config.Voltage.PeakForwardVoltage = 12.0;
            config.Voltage.PeakReverseVoltage = -12.0;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Swerve.OPEN_LOOP_RAMP;
            config.OpenLoopRamps.VoltageOpenLoopRampPeriod = Swerve.OPEN_LOOP_RAMP;
            return config;
        }

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FrontLeftMod { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 8;
            public static final int ANGLE_MOTOR_ID = 10;
            public static final int CANCODER_ID = 7;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(0.142334);
            public static final boolean IS_DRIVE_INVERTED = true;
            public static final boolean IS_ANGLE_INVERTED = false;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, IS_DRIVE_INVERTED, IS_ANGLE_INVERTED);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRightMod { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 9;
            public static final int ANGLE_MOTOR_ID = 11;
            public static final int CANCODER_ID = 6;
            public static final boolean IS_DRIVE_INVERTED = true;
            public static final boolean IS_ANGLE_INVERTED = false;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(0.427246);
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, IS_DRIVE_INVERTED, IS_ANGLE_INVERTED);
        }
        
        /* Back Left Module - Module 2 */
        public static final class BackLeftMod { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 5;
            public static final int CANCODER_ID = 0;
            public static final boolean IS_DRIVE_INVERTED = true;
            public static final boolean IS_ANGLE_INVERTED = false;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(0.174316);
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, IS_DRIVE_INVERTED, IS_ANGLE_INVERTED);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRightMod { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final int CANCODER_ID = 1;
            public static final boolean IS_DRIVE_INVERTED = false;
            public static final boolean IS_ANGLE_INVERTED = false;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(0.413330);
            public static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET, IS_DRIVE_INVERTED, IS_ANGLE_INVERTED);
        }
    }

    public static final class LED { //TODO: This must be tuned to the specific robot
        public static final int LED_START = 0;
        public static final int LED_LENGTH =  117;
        public static final int LED_PORT_ID = 3;
    }

    public static final class Auto { //TODO: This must be tuned to specific robot
        public static final double X_CONTROLLER = 6;
        public static final double Y_CONTROLLER = 6;
        public static final double THETA_CONTROLLER = 4;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                Swerve.MAX_ANGULAR_VELOCITY, Swerve.MAX_ANGULAR_ACCELERATION);
    }

    public static class Limelight { //TODO: this must be tuned to specific robot
        public static VisionDeviceConstants LEFT_VISION_DEVICE = new VisionDeviceConstants();
        public static VisionDeviceConstants RIGHT_VISION_DEVICE = new VisionDeviceConstants();
        public static VisionDeviceConstants FRONT_VISION_DEVICE = new VisionDeviceConstants();
        public static VisionDeviceConstants BACK_VISION_DEVICE = new VisionDeviceConstants();

        static {
            LEFT_VISION_DEVICE.kTableName = "limelight-left";    
            LEFT_VISION_DEVICE.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                    new Translation2d(Units.inchesToMeters(10.5), Units.inchesToMeters(1.23)),
                    Rotation2d.fromDegrees(-90));

            RIGHT_VISION_DEVICE.kTableName = "limelight-right";  
            RIGHT_VISION_DEVICE.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                    new Translation2d(Units.inchesToMeters(10.78), Units.inchesToMeters(2)),
                    Rotation2d.fromDegrees(90));
            
            FRONT_VISION_DEVICE.kTableName = "limelight-front";
            FRONT_VISION_DEVICE.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                    new Translation2d(Units.inchesToMeters(11.11), Units.inchesToMeters(4.28)),
                    Rotation2d.fromDegrees(0));

            BACK_VISION_DEVICE.kTableName = "limelight-back";
            BACK_VISION_DEVICE.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                    new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(-0.96)),
                    Rotation2d.fromDegrees(180));
        }

    }
    public static final class PathPlannerRobotConfig {
        public static RobotConfig CONFIG = null;
        static {
            try {
                CONFIG = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                System.err.println("Pathplanner Configs failed to load!");
            }
        }
    }

    // Timeout constants
	public static final int LONG_CANT_TIMEOUT_MS = 100;
}
