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
import frc.robot.lib.swerve.COTSTalonFXSwerveConstants;
import frc.robot.lib.swerve.SwerveModuleConstants;
import frc.robot.subsystems.Drive.KinematicLimits;
import frc.robot.subsystems.vision.VisionDeviceConstants;

public final class Constants {
    public static final double stickDeadband = 0.07;
    public static boolean isEpsilon;

    // robot loop time
	public static final double kLooperDt = 0.02;
    // Disables extra smart dashboard outputs that slow down the robot
	public static final boolean disableExtraTelemetry = false;

    public static final class Swerve {
        public static final int pigeonID = 20;

        public static final COTSTalonFXSwerveConstants chosenModule =  
            COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24); 
        public static final double wheelBase = Units.inchesToMeters(24);
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double wheelDiameter = Swerve.chosenModule.wheelDiameter; 

        /* Swerve Kinematics */
        public static final Translation2d[] swerveModuleLocations = {
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };
        
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            swerveModuleLocations[0],
            swerveModuleLocations[1],
            swerveModuleLocations[2],
            swerveModuleLocations[3]
        );

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 20;
        public static final int angleCurrentThreshold = 30;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 30;
        public static final int driveCurrentThreshold = 45; 
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 15.0;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0004;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4;
        public static final double maxAutoSpeed = maxSpeed * 0.85;  // Max out at 85% to ensure attainable speeds, 

        /** Meters per Second Squared */
        public static final double maxAcceleration = 3;
        /** Radians per Second */
        public static final double maxAngularVelocity = Math.PI * 3; //TODO: This must be tuned to specific robot

        /** Radians per Second Squared */
        public static final double kMaxAngularAcceleration = 720.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;


        public static final boolean invertGyro = false; // TODO: ensure Gyro is CCW+ CW-

        /* Heading Controller */

        // Stabilize Heading PID Values
        public static final double kStabilizeSwerveHeadingKp = 10.0;
        public static final double kStabilizeSwerveHeadingKi = 0.0;
        public static final double kStabilizeSwerveHeadingKd = 0.3;
        public static final double kStabilizeSwerveHeadingKf = 2.0;

        // Snap Heading PID Values
        public static final double kSnapSwerveHeadingKp = 10.0;
        public static final double kSnapSwerveHeadingKi = 0.0;
        public static final double kSnapSwerveHeadingKd = 0.6;
        public static final double kSnapSwerveHeadingKf = 1.0;

        public static final SwerveDriveKinematics kKinematics = Swerve.swerveKinematics;

        public static final double kV = 12 * Math.PI * wheelDiameter / (driveGearRatio * maxSpeed);
        
        public static final double kCancoderBootAllowanceSeconds = 10.0;

        public static final KinematicLimits kUncappedLimits = new KinematicLimits();

        static {
            kUncappedLimits.kMaxDriveVelocity = maxSpeed;
            kUncappedLimits.kMaxAccel = Double.MAX_VALUE;
            kUncappedLimits.kMaxAngularVelocity = Swerve.maxAngularVelocity;
            kUncappedLimits.kMaxAngularAccel = Double.MAX_VALUE;
        }

        /* TalonFx module constants*/
        public static TalonFXConfiguration AzimuthFXConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.Slot0.kP = 10.0;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 1.0;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;

            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = Swerve.angleCurrentLimit;

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = Swerve.angleCurrentLimit;
            config.CurrentLimits.SupplyCurrentLowerLimit = Swerve.angleCurrentThreshold;
            config.CurrentLimits.SupplyCurrentLowerTime = Swerve.angleCurrentThresholdTime;

            config.Voltage.PeakForwardVoltage = 12.0;
            config.Voltage.PeakReverseVoltage = -12.0;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            return config;
        }

        public static TalonFXConfiguration DriveFXConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.Slot0.kP = 1.2;
            config.Slot0.kI = 0.005;
            config.Slot0.kD = 0.0;
            config.Slot0.kS = 0.1;
            config.Slot0.kV = 12 * Math.PI * wheelDiameter / (driveGearRatio * maxSpeed);

            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = Swerve.driveCurrentLimit;

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = Swerve.driveCurrentLimit;
            config.CurrentLimits.SupplyCurrentLowerLimit = Swerve.driveCurrentThreshold;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.5;

            config.Voltage.PeakForwardVoltage = 12.0;
            config.Voltage.PeakReverseVoltage = -12.0;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Swerve.openLoopRamp;
            config.OpenLoopRamps.VoltageOpenLoopRampPeriod = Swerve.openLoopRamp;
            return config;
        }

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FrontLeftMod { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.142334);
            public static final boolean isDriveInverted = true;
            public static final boolean isAngleInverted = false;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, isDriveInverted, isAngleInverted);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRightMod { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 6;
            public static final boolean isDriveInverted = true;
            public static final boolean isAngleInverted = false;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.427246);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, isDriveInverted, isAngleInverted);
        }
        
        /* Back Left Module - Module 2 */
        public static final class BackLeftMod { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 0;
            public static final boolean isDriveInverted = true;
            public static final boolean isAngleInverted = false;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.174316);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, isDriveInverted, isAngleInverted);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRightMod { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final boolean isDriveInverted = false;
            public static final boolean isAngleInverted = false;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.413330);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, isDriveInverted, isAngleInverted);
        }

    }

    public static final class Auto { //TODO: This must be tuned to specific robot
        public static final double kPXController = 6;
        public static final double kPYController = 6;
        public static final double kPThetaController = 4;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                Swerve.maxAngularVelocity, Swerve.kMaxAngularAcceleration);
    }

    public static class Limelight { //TODO: this must be tuned to specific robot
        public static VisionDeviceConstants kLeftVisionDevice = new VisionDeviceConstants();
        public static VisionDeviceConstants kRightVisionDevice = new VisionDeviceConstants();
        public static VisionDeviceConstants kFrontVisionDevice = new VisionDeviceConstants();
        public static VisionDeviceConstants kBackVisionDevice = new VisionDeviceConstants();

        static {
            kLeftVisionDevice.kTableName = "limelight-left";    
            kLeftVisionDevice.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                    new Translation2d(Units.inchesToMeters(10.5), Units.inchesToMeters(1.23)),
                    Rotation2d.fromDegrees(-90));

            kRightVisionDevice.kTableName = "limelight-right";  
            kRightVisionDevice.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                    new Translation2d(Units.inchesToMeters(10.78), Units.inchesToMeters(2)),
                    Rotation2d.fromDegrees(90));
            
            kFrontVisionDevice.kTableName = "limelight-front";
            kFrontVisionDevice.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                    new Translation2d(Units.inchesToMeters(11.11), Units.inchesToMeters(4.28)),
                    Rotation2d.fromDegrees(0));

            kBackVisionDevice.kTableName = "limelight-back";
            kBackVisionDevice.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                    new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(-0.96)),
                    Rotation2d.fromDegrees(180));
        }

    }
    public static final class PathPlannerRobotConfig {
        public static RobotConfig config = null;
        static {
            try {
                config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                System.err.println("Pathplanner Configs failed to load!");
                e.printStackTrace();
            }
        }
    }
    
    public static final class LEDS { //TODO: This must be tuned to the specific robot
        public static final int ledStart = 0;
        public static final int ledLength =  117;
    }

    // Timeout constants
	public static final int kLongCANTimeoutMs = 100;
}
