package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.RobotConfig;
//dc.10.21.2024 classes used in SwerveConstants
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.lib.util.COTSTalonFXSwerveConstants;
import frc.robot.lib.util.SwerveModuleConstants;
import frc.robot.subsystems.SwerveDrive.KinematicLimits;
import frc.robot.subsystems.limelight.GoalTracker;
import frc.robot.subsystems.vision.VisionDeviceConstants;
import com.pathplanner.lib.config.RobotConfig;

public final class Constants {
    public static final double stickDeadband = 0.07;
    public static boolean isEpsilon;
    public static boolean isBareboneRobot=true; //dc.10.29.2024, set to true for barebone robot, false for full robot//

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

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final Translation2d[] swerveModuleLocations = {  //dc.10.28.2024, need for WheelTracker.java
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
         };
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            swerveModuleLocations[0],
            swerveModuleLocations[1],
            swerveModuleLocations[2],
            swerveModuleLocations[3]);//DC.11.14.24 shall be the 4th wheel 

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

        public static final int driveCurrentLimit = 30; //dc.11.9.24 reduce max current per motor, total current needs to time motor-count(8)
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
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0004;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4; //TODO: dc 11.9.24, increase max speed so that we can observe amplified drivetrain bahavior 
        /** Radians per Second */
        public static final double maxAngularVelocity = 3.14 * 2; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FrontLeftMod { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.142334);//0.147705);//0.142334);
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
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.427246);//430420);//0.419434);
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
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.174316);//173340);//0.175781);
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
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.413330);//416748);//0.417236);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, isDriveInverted, isAngleInverted);
        }

        public static double kMaxAngularAcceleration = 720.0; //TODO: set this to tuned value in future
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static VisionDeviceConstants kLeftVisionDevice = new VisionDeviceConstants(); // dot 13
    public static VisionDeviceConstants kRightVisionDevice = new VisionDeviceConstants(); // dot 12
    public static VisionDeviceConstants kFrontVisionDevice = new VisionDeviceConstants();
    public static VisionDeviceConstants kBackVisionDevice = new VisionDeviceConstants();

        static {//dc.2.10.25, TODO: update camera setting according to robot h/w config
        kLeftVisionDevice.kTableName = "limelight-left";    
        kLeftVisionDevice.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                new Translation2d(Units.inchesToMeters(10.5), Units.inchesToMeters(1.23)),
                Rotation2d.fromDegrees(-90));

        kRightVisionDevice.kTableName = "limelight-right";  
        kRightVisionDevice.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                new Translation2d(Units.inchesToMeters(10.78), Units.inchesToMeters(2)),
                Rotation2d.fromDegrees(90));
        
        kFrontVisionDevice.kTableName = "limelight-frontl";
        kFrontVisionDevice.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                new Translation2d(Units.inchesToMeters(11.11), Units.inchesToMeters(4.28)),
                Rotation2d.fromDegrees(0));

        kBackVisionDevice.kTableName = "limelight-frontr";
        kBackVisionDevice.kRobotToCamera = new edu.wpi.first.math.geometry.Transform2d(
                new Translation2d(0.308, 0.251),
                Rotation2d.fromDegrees(25));
}

    //dc.10.21.2024, citrus code constants
    public static final class SwerveConstants {
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

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

        /*dc.10.21.2024 mapping existing constants so that ported citrus code only needs minimal changes */
        public static final SwerveDriveKinematics kKinematics = Swerve.swerveKinematics;
        //public static final boolean driveMotorInvert = false;   //TODO: need to verify with the actual Robot setting
        //public static final boolean angleMotorInvert = true;    //TODO: need to verify with the actual Robot setting
        public static final double wheelDiameter = Swerve.chosenModule.wheelDiameter; //??4.0inch
        public static final double wheelCircumference = Swerve.chosenModule.wheelCircumference;
        public static final double driveGearRatio = Swerve.chosenModule.driveGearRatio;//?? Constants.isEpsilon ? 5.82 : 5.82; 
        public static final double angleGearRatio = Swerve.chosenModule.angleGearRatio;
        public static final double maxSpeed = Swerve.maxSpeed; 
        public static final double maxAngularVelocity = Swerve.maxAngularVelocity;
        public static final double kV = 12 * Math.PI * wheelDiameter / (driveGearRatio * maxSpeed); //TODO: need to finetune with the actual robot
        public static final double maxAutoSpeed = maxSpeed * 0.85;  // Max out at 85% to ensure attainable speeds, 
        
        public static final double kCancoderBootAllowanceSeconds = 10.0;

        public static final KinematicLimits kUncappedLimits = new KinematicLimits();

		static {
			kUncappedLimits.kMaxDriveVelocity = maxSpeed;
			kUncappedLimits.kMaxAccel = Double.MAX_VALUE;
			kUncappedLimits.kMaxAngularVelocity = Swerve.maxAngularVelocity;
			kUncappedLimits.kMaxAngularAccel = Double.MAX_VALUE;
		}

        public static final boolean invertYAxis = false;
		public static final boolean invertRAxis = false;
		public static final boolean invertXAxis = false;

        /* TalonFx module constants*/
        
        public static TalonFXConfiguration AzimuthFXConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            //dc.2.23.25, tuning steering motor to get rid of drifting (mainly rotation)
            config.Slot0.kP = 0.09;//.3;        //Proportional compensation: small P value to keep motor from drifting, big value will cause oscillation
            config.Slot0.kI = 0.0; //0.0007;//0.01;    //Integral compensation: a small I value to help to quickly turn
            config.Slot0.kD = 0.0016;//0.0008;  //Derivative compenstion: damping oscillation
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;

            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = Swerve.angleCurrentLimit;//80;

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = Swerve.angleCurrentLimit;//60;
            config.CurrentLimits.SupplyCurrentLowerLimit = Swerve.angleCurrentThreshold;//dc:  ctre updates 2025 
            config.CurrentLimits.SupplyCurrentLowerTime = Swerve.angleCurrentThresholdTime; ;//dc:  ctre updates 2025

            config.Voltage.PeakForwardVoltage = 12.0;
            config.Voltage.PeakReverseVoltage = -12.0;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            return config;
        }

        public static TalonFXConfiguration DriveFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.Slot0.kP = 0.030 * 12.0;
			config.Slot0.kI = 0.0;
			config.Slot0.kD = 0.000000 * 12.0;
			config.Slot0.kS = 0.1;
			config.Slot0.kV = 12 * Math.PI * wheelDiameter / (driveGearRatio * maxSpeed);

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = Swerve.driveCurrentLimit;//citrus code value = 110;

			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyCurrentLimit = Swerve.driveCurrentLimit;//citrus value = 90;
            config.CurrentLimits.SupplyCurrentLowerLimit = Swerve.driveCurrentThreshold;//add this to limit current spiking 
			config.CurrentLimits.SupplyCurrentLowerTime = 0.5;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

			config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Swerve.openLoopRamp;
			config.OpenLoopRamps.VoltageOpenLoopRampPeriod = Swerve.openLoopRamp;
			return config;
		}

    }

    //dc.2.11.2025, retain the current elevator code in main branch while merging with strategybranch
    public static class Elevator {
        //TODO: tune elevator constants to bot
        public static final int kElevatorLeftMotorId = 20;
        public static final int kElevatorRightMotorId = 21;
        
        public static final double kG = 0.1;
        public static final double kS = 0.125;
        public static final double kV = 0.0;
        public static final double kP = 5.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        
    
        public static final double kCruiseVelocity = 40;
        public static final double kAcceleration = 72.5;
        public static final double kJerk = 1600;
        public static final int CurrentThreshold = 45;
        public static final int kMaxCurrent = 40;
        public static final double kMaxPowerUp = 0.1;
        public static final double kMaxPowerDown = 0.1;
        
        //TODO: Find correct elevator heights for each level
        public static final double kGroundHeight = 0.5; //occasionally stalls at bottom
        public static final double kL2Height = 9.15;
        public static final double kL3Height = 22;
        
        public static final double kL4Height = 43.15;    //stalls at top
        public static final double kAPHeight = 6.9;
        public static final double kA1Height = 25.7;  //19.4
        public static final double kA2Height = 32.5; //Unsure
        public static final double KDefaultHeight = 13;// this height clears the reef but not blocking the front camera;
        
        public static final TalonFXConfiguration ElevatorConfiguration() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = Constants.Elevator.kMaxCurrent;//citrus code value = 110;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.kMaxCurrent;//citrus value = 90;
            config.Voltage.PeakForwardVoltage = 12.0;
            config.Voltage.PeakReverseVoltage = -12.0;
            // Set PID values for the elevator motor
            config.Slot0.kP = Constants.Elevator.kP;
            config.Slot0.kI = Constants.Elevator.kI;
            config.Slot0.kD = Constants.Elevator.kD;
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            return config;
        }
    }
    
    public static class Funnel {
        //TODO: Tune intake constants to bot

        // Motors
        public static final int kPivotMotorId = 50;
    
        // DIO
        public static final int kIntakeLimitSwitchId = 30;
        public static final int kShooterLimitSwitchId = 31;
    
    
        // Absolute encoder offset
        public static final double k_pivotEncoderOffset = 0.166842; // Straight up, sketchy to reset to "up"
    
        // Pivot set point angles
        //public static final double k_pivotAngleGround = 60;
        //public static final double k_pivotAngleSource = 190;
        //public static final double k_pivotAngleAmp = k_pivotAngleSource;
        //public static final double k_pivotAngleStow = 275;
    
        // Intake speeds
        public static final double k_pivotStartAngle = 0;
        public static final double k_pivotEndAngle = 75;
        
        public static final TalonFXConfiguration IntakeConfiguration() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 10;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 10;
            config.Voltage.PeakForwardVoltage = 12.0;
            config.Voltage.PeakReverseVoltage = -12.0;
            return config;
        }
      }

      public static class CoralShooter {    //originally Shooter
        public static final int kShooterLeftMotorId = 12;
        public static final int kShooterRightMotorId = 13;
        
        public static final double kShooterIntakeSpeed = 0.05;
        public static final double kShooterShootSpeed = 0.175;
        ;
        
      }

      //dc.2.11.25, keey the shooter class for now, TODO: remove when CoralShooter is QAed.
      public static class Shooter {
        public static final int kShooterLeftMotorId = 12;
        public static final int kShooterRightMotorId = 13;

        public static final double kShooterP = 0.00005;
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.0;
        public static final double kShooterFF = 0.0002;

        public static final double kShooterMinOutput = 0;
        public static final double kShooterMaxOutput = 1;
        public static final TalonFXConfiguration ShooterConfiguration() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.Slot0.kP = Constants.Shooter.kShooterP;
            config.Slot0.kI = Constants.Shooter.kShooterI;
            config.Slot0.kD = Constants.Shooter.kShooterD;
            return config;
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

    public static final class AlgaeShooter { //TODO: make constants correct
        public static final int kAlgaePivotMotorId = 26;
        public static final int kAlgaeShooterMotorId = 28;

        public static final double kAlgaeShooterSpeed = 0.4;
        public static final double kIntakePosition = 5.93;
        public static final double kBargePosition = 0;
        public static final double kProcessorPosition = 4.5;
        public static final double kRestingPosition = 0;
        public static final double kGroundPosition = 5.93;

        public static final double kS = 0.015;
        public static final double kV = 0.0;
        public static final double kP = 0.25;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        
    
        public static final double kCruiseVelocity = 2;
        public static final double kAcceleration = 5;
        public static final double kJerk = 1600;
        public static final double kWaitTime = 0.5;
    }

    public static final class Hang { //TODO: make constants correct
        public static final int kHangMotorId = 51;
        public static final double kHangSpeed = 0.05;
        public static final double kHoldSpeed = 0.02;
    }
    
    public static final class LEDS { //TODO: make constants correct
        public static final int ledStart = 0;
        public static final int ledLength =  117;
    }


    /* dc.10.21.2024 extra constants needed during porting of citrus SwerveModule.java code */

    // Timeout constants
	public static final int kLongCANTimeoutMs = 100;
}
