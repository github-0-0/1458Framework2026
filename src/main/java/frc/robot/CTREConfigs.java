package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs() {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.CANCODER_INVERT;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.ANGLE_MOTOR_INVERT;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.ANGLE_NEUTRAL_MODE;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.ANGEL_GEAR_RATIO;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.ANGLE_ENABLE_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.ANGLE_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.ANGLE_CURRENT_THRESHOLD;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.ANGLE_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.ANGLE_KP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.ANGLE_KI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.ANGLE_KD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.DRIVE_MOTOR_INVERT;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.DRIVE_NEUTRAL_MODE;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.DRIVE_GEAR_RATIO;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.DRIVE_ENABLE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.DRIVE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.DRIVE_CURRENT_THRESHOLD;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.DRIVE_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.DRIVE_KP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.DRIVE_KI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.DRIVE_KD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;
    }
}