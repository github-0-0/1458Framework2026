package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;



import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;


import frc.robot.Constants;
import frc.robot.lib.util.Conversions;
import frc.robot.lib.util.Util;
import frc.robot.lib.drivers.Phoenix6Util;
import frc.robot.lib.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class Module extends Subsystem {
	private final int kModuleNumber;
	private final double kAngleOffset;

	private TalonFX mAngleMotor;
	private TalonFX mDriveMotor;
	private CANcoder angleEncoder;
	private DCMotorSim mDriveMotorSim;
	private DCMotorSim mAngleMotorSim;

	private BaseStatusSignal[] mSignals = new BaseStatusSignal[4];

	private mPeriodicIO mPeriodicIO = new mPeriodicIO();

	public static class mPeriodicIO {
		// Inputs
		public double timestamp = 0.0;
		public double rotationPosition = 0.0;
		public double rotationVelocity = 0.0;
		public double drivePosition = 0.0;
		public double driveVelocity = 0.0;

		// Outputs
		public double targetVelocity = 0.0;
		public ControlRequest rotationDemand;
		public ControlRequest driveDemand;
	}


	public Module(int moduleNumber, SwerveModuleConstants moduleConstants, CANcoder cancoder) {
		this.kModuleNumber = moduleNumber;
		kAngleOffset = moduleConstants.angleOffset.getDegrees();	//kAngleOffset is in radians

		angleEncoder = cancoder;

		// Angle motor config
		mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "CV");
		Phoenix6Util.checkErrorAndRetry(() ->
				mAngleMotor.getConfigurator().apply(Constants.Swerve.AzimuthFXConfig(), Constants.kLongCANTimeoutMs));
		mAngleMotor.setInverted(moduleConstants.angleInvert);

		// Drive motor config
		mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "CV");
		Phoenix6Util.checkErrorAndRetry(() ->
				mDriveMotor.getConfigurator().apply(Constants.Swerve.DriveFXConfig(), Constants.kLongCANTimeoutMs));
		mDriveMotor.setInverted(moduleConstants.driveInvert);
		mDriveMotor.setPosition(0.0);

		resetToAbsolute();

		mSignals[0] = mDriveMotor.getRotorPosition();
		mSignals[1] = mDriveMotor.getRotorVelocity();
		mSignals[2] = mAngleMotor.getRotorPosition();
		mSignals[3] = mAngleMotor.getRotorVelocity();


		mDriveMotorSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getKrakenX60Foc(1), 0.001, Constants.Swerve.driveGearRatio),
				DCMotor.getKrakenX60Foc(1));
		mAngleMotorSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getKrakenX60Foc(1), 0.001, Constants.Swerve.angleGearRatio),
				DCMotor.getFalcon500Foc(1));
	}

	@Override
	public synchronized void readPeriodicInputs() {
		mPeriodicIO.timestamp = Timer.getFPGATimestamp();
		refreshSignals();

		//check drive and angle motor's current and voltage and publish to NetworkTable, plot them in SIM GUI to verify motion profile used by TalonFx motor
		//TODO: clean up at production release
		double driveMotorStatorCurrent = mDriveMotor.getStatorCurrent().getValueAsDouble();
		double driveMotorVoltage = mDriveMotor.getMotorVoltage().getValueAsDouble();
		SmartDashboard.putNumber("Drive/Module#" + kModuleNumber +"/DriveMotor/Voltage", driveMotorVoltage);
		SmartDashboard.putNumber("Drive/Module#" + kModuleNumber +"/DriveMotor/Current", driveMotorStatorCurrent);

		double angleMotorStatorCurrent = mAngleMotor.getStatorCurrent().getValueAsDouble();
		double angleMotorVoltage = mAngleMotor.getMotorVoltage().getValueAsDouble();
		SmartDashboard.putNumber("Drive/Module#" + kModuleNumber +"/AngleMotor/Voltage", angleMotorVoltage);
		SmartDashboard.putNumber("Drive/Module#" + kModuleNumber +"/AngleMotor/Current", angleMotorStatorCurrent);
	}

	public synchronized void refreshSignals() {
		//TODO: might need to add call to StatusSignal.refresh() for each signal in reading before get value
 		mPeriodicIO.rotationVelocity = mAngleMotor.getRotorVelocity().getValueAsDouble();
		mPeriodicIO.driveVelocity = mDriveMotor.getRotorVelocity().getValueAsDouble();

		mPeriodicIO.rotationPosition = BaseStatusSignal.getLatencyCompensatedValueAsDouble(//dc: update to 2025 APIs
				mAngleMotor.getRotorPosition(), mAngleMotor.getRotorVelocity());
		mPeriodicIO.drivePosition = mDriveMotor.getRotorPosition().getValueAsDouble();
	}

	public void setOpenLoop(SwerveModuleState desiredState) {
		double flip = setSteeringAngleOptimized(new Rotation2d(desiredState.angle.getRadians())) ? -1 : 1;
		mPeriodicIO.targetVelocity = desiredState.speedMetersPerSecond * flip;
		double rotorSpeed = Conversions.MPSToRPS(
				mPeriodicIO.targetVelocity, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
		mPeriodicIO.driveDemand = new VoltageOut(rotorSpeed * Constants.Swerve.kV)
				.withEnableFOC(true)
				.withOverrideBrakeDurNeutral(false);
	}

	public void setVelocity(SwerveModuleState desiredState) {
		double flip = setSteeringAngleOptimized(new Rotation2d(desiredState.angle.getRadians())) ? -1 : 1; //dc: modify to support WPILib Rotation2d constructor
		mPeriodicIO.targetVelocity = desiredState.speedMetersPerSecond * flip;
		double rotorSpeed = Conversions.MPSToRPS(
				mPeriodicIO.targetVelocity,
				Constants.Swerve.wheelCircumference,
				Constants.Swerve.driveGearRatio);

		if (Math.abs(rotorSpeed) < 0.002) {
			mPeriodicIO.driveDemand = new NeutralOut();
		} else {
			mPeriodicIO.driveDemand = new VelocityVoltage(rotorSpeed);
		}
	}


	private boolean setSteeringAngleOptimized(Rotation2d steerAngle) {
		boolean flip = false;
		final double targetClamped = - steerAngle.getDegrees();//See comments above for the negate operation
		final double angleUnclamped = getCurrentUnboundedDegrees();
		final Rotation2d angleClamped = Rotation2d.fromDegrees(angleUnclamped);
		final Rotation2d relativeAngle = Rotation2d.fromDegrees(targetClamped).rotateBy(angleClamped.unaryMinus());
		double relativeDegrees = relativeAngle.getDegrees();
 		if (relativeDegrees > 90.0) {
			relativeDegrees -= 180.0;
			flip = true;

		} else if (relativeDegrees < -90.0) {
			relativeDegrees += 180.0;
			flip = true;
		}
		setSteeringAngleRaw(angleUnclamped + relativeDegrees);
		target_angle = angleUnclamped + relativeDegrees;
		return flip;
	}

	private double target_angle;

	private void setSteeringAngleRaw(double angleDegrees) {
		double rotorPosition = Conversions.degreesToRotation(angleDegrees, Constants.Swerve.angleGearRatio);
		mPeriodicIO.rotationDemand = new PositionDutyCycle(rotorPosition);
		SmartDashboard.putNumber("Drive/Module#" + kModuleNumber +"/AngleMotor/DemandAngle", angleDegrees);
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		mAngleMotor.setControl(mPeriodicIO.rotationDemand);
		mDriveMotor.setControl(mPeriodicIO.driveDemand);
	}


	public void resetToAbsolute() {
 		angleEncoder.getAbsolutePosition().waitForUpdate(Constants.kLongCANTimeoutMs);
		double angle = Util.placeInAppropriate0To360Scope(
				getCurrentUnboundedDegrees(), -(getCanCoder().getDegrees() - kAngleOffset));
		double absolutePosition = Conversions.degreesToRotation(angle, Constants.Swerve.angleGearRatio);
		Phoenix6Util.checkErrorAndRetry(() -> mAngleMotor.setPosition(absolutePosition, Constants.kLongCANTimeoutMs));
	}

	public void setDriveNeutralBrake(boolean wantBrake) {
		TalonFXConfiguration t = new TalonFXConfiguration();
		mDriveMotor.getConfigurator().refresh(t);
		t.MotorOutput.NeutralMode = wantBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		mDriveMotor.getConfigurator().apply(t);
	}

	@Override
	public void outputTelemetry() {
		// spotless:off
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Azi Target", target_angle);
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Azi Angle", getCurrentUnboundedDegrees());
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Azi Error", getCurrentUnboundedDegrees() - target_angle);
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Wheel Velocity", Math.abs(getCurrentVelocity()));
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Wheel Target Velocity", Math.abs(mPeriodicIO.targetVelocity));
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Drive Position", Math.abs(mPeriodicIO.drivePosition));
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Duty Cycle",
				mDriveMotor.getDutyCycle().getValueAsDouble());
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Azi Current",
				mAngleMotor.getSupplyCurrent().getValueAsDouble());
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Drive Current",
				mDriveMotor.getSupplyCurrent().getValueAsDouble());
		SmartDashboard.putNumber("Module" + kModuleNumber + "/Wheel Velocity Error",
				Math.abs(getCurrentVelocity()) - Math.abs(mPeriodicIO.targetVelocity));
		// spotless:on
	}

	public int moduleNumber() {
		return kModuleNumber;
	}

	public double angleOffset() {
		return kAngleOffset;
	}

	public Rotation2d getCanCoder() {
		return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);
	}

	public Rotation2d getModuleAngle() {
		return Rotation2d.fromDegrees(getCurrentUnboundedDegrees());
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getCurrentVelocity(), getModuleAngle());
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getDriveDistanceMeters(), getModuleAngle());
	}

	public edu.wpi.first.math.kinematics.SwerveModulePosition getWpiPosition() {
		return new edu.wpi.first.math.kinematics.SwerveModulePosition(
				getDriveDistanceMeters(),
				edu.wpi.first.math.geometry.Rotation2d.fromDegrees(
						getModuleAngle().getDegrees()));
	}

	public double getTargetVelocity() {
		return mPeriodicIO.targetVelocity;
	}

	public double getCurrentVelocity() {
		return Conversions.RPSToMPS(
				mPeriodicIO.driveVelocity,
				Constants.Swerve.wheelCircumference,
				Constants.Swerve.driveGearRatio);
	}

	public double getDriveDistanceMeters() {
		return Conversions.rotationsToMeters(
				mPeriodicIO.drivePosition,
				Constants.Swerve.wheelCircumference,
				Constants.Swerve.driveGearRatio);
	}

	public double getCurrentUnboundedDegrees() {
		return Conversions.rotationsToDegrees(mPeriodicIO.rotationPosition, Constants.Swerve.angleGearRatio);
	}

	public double getTimestamp() {
		return mPeriodicIO.timestamp;
	}

	public double getDriveMotorCurrent() {
		return mDriveMotor.getStatorCurrent().getValueAsDouble();
	}

	public BaseStatusSignal[] getUsedStatusSignals() {
		return mSignals;
	}

/*     //TODO: need to reconcile the following class with lib.util.SwerveModuleConstants class
	public static class SwerveModuleConstants {
		public final int driveMotorID;
		public final int angleMotorID;
		public final double angleOffset;

		public SwerveModuleConstants(int driveMotorID, int angleMotorID, double angleOffset) {
			this.driveMotorID = driveMotorID;
			this.angleMotorID = angleMotorID;
			this.angleOffset = angleOffset;
		}
	}
*/
	public void updateSimPeriodic() {
		TalonFXSimState mDriveMotorSimState = mDriveMotor.getSimState();
		TalonFXSimState mAngleMotorSimState = mAngleMotor.getSimState();
		CANcoderSimState angleEncoderSimState = angleEncoder.getSimState();

		// Pass the robot battery voltage to the simulated devices
		mDriveMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
		mAngleMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
		angleEncoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

		// Simulate drive
		mDriveMotorSim.setInputVoltage(mDriveMotorSimState.getMotorVoltageMeasure().in(Volts));
		mDriveMotorSim.update(TimedRobot.kDefaultPeriod);

		mDriveMotorSimState.setRawRotorPosition(
			mDriveMotorSim.getAngularPosition().times(Constants.Swerve.driveGearRatio));
		mDriveMotorSimState.setRotorVelocity(
			mDriveMotorSim.getAngularVelocity().times(Constants.Swerve.driveGearRatio));

		// Simulate steering
		mAngleMotorSim.setInputVoltage(mAngleMotorSimState.getMotorVoltageMeasure().in(Volts));
		mAngleMotorSim.update(TimedRobot.kDefaultPeriod);

		mAngleMotorSimState.setRawRotorPosition(
			mAngleMotorSim.getAngularPosition().times(Constants.Swerve.angleGearRatio));
		mAngleMotorSimState.setRotorVelocity(
			mAngleMotorSim.getAngularVelocity().times(Constants.Swerve.angleGearRatio));

		angleEncoderSimState.setRawPosition(
			mAngleMotorSim.getAngularPosition().times(Constants.Swerve.angleGearRatio));
		angleEncoderSimState.setVelocity(
			mAngleMotorSim.getAngularVelocity().times(Constants.Swerve.angleGearRatio));
	}
}
