package frc.robot.lib.swerve;

//dc.10.21.2024 ported from com.team1678.lib.swerve;

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
import com.ctre.phoenix6.sim.TalonFXSimState;

//dc.10.21.2024 replacing with our own/ported classes
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Subsystem;
import frc.robot.lib.util.Conversions;
import frc.robot.lib.util.Util;
import frc.robot.lib.drivers.Phoenix6Util;
import frc.robot.lib.util.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//dc.10.21.2024, replace citrus SwerveModuleState with WPILIB version, the same practice as other Victor & Shaji
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


public class SwerveModule extends Subsystem {
	private final int kModuleNumber;
	private final double kAngleOffset;

	private TalonFX mAngleMotor;
	private TalonFX mDriveMotor;
	private CANcoder angleEncoder;
	private DCMotorSim mDriveMotorSim;
	private DCMotorSim mAngleMotorSim;

	private BaseStatusSignal[] mSignals = new BaseStatusSignal[4];

	private mPeriodicIO mPeriodicIO = new mPeriodicIO();

	private int mCounter = 0;//TODO: code for debug, to be removed

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

	//dc.10.25.2024 replace citrus SwerveModuleConstants with our own. Just need to angleOffset.getRadians, and disregard CancoderID
	public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, CANcoder cancoder) {
		this.kModuleNumber = moduleNumber;
		kAngleOffset = moduleConstants.angleOffset.getDegrees();	//kAngleOffset is in radians

		angleEncoder = cancoder;

		// Angle motor config
		mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "CV");
		Phoenix6Util.checkErrorAndRetry(() ->
				mAngleMotor.getConfigurator().apply(SwerveConstants.AzimuthFXConfig(), Constants.kLongCANTimeoutMs));
		mAngleMotor.setInverted(moduleConstants.angleInvert);

		// Drive motor config
		mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "CV");
		Phoenix6Util.checkErrorAndRetry(() ->
				mDriveMotor.getConfigurator().apply(SwerveConstants.DriveFXConfig(), Constants.kLongCANTimeoutMs));
		mDriveMotor.setInverted(moduleConstants.driveInvert);
		mDriveMotor.setPosition(0.0);

		resetToAbsolute();

		mSignals[0] = mDriveMotor.getRotorPosition();
		mSignals[1] = mDriveMotor.getRotorVelocity();
		mSignals[2] = mAngleMotor.getRotorPosition();
		mSignals[3] = mAngleMotor.getRotorVelocity();

	/*dc: Todo: update motor sim according to CTRE 2025 APIs 
		mDriveMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), Constants.Swerve.driveGearRatio, 0.001);
		mAngleMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), Constants.Swerve.angleGearRatio, 0.001);
	*/
		mDriveMotorSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getFalcon500(1), 0.001, Constants.Swerve.driveGearRatio),
				DCMotor.getFalcon500(1));
		mAngleMotorSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getFalcon500(1), 0.001, Constants.Swerve.angleGearRatio),
				DCMotor.getFalcon500(1));
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
		double flip = setSteeringAngleOptimized(new Rotation2d(desiredState.angle.getRadians())) ? -1 : 1;  //dc: modify to support WPILib Rotation2d constructor
		mPeriodicIO.targetVelocity = desiredState.speedMetersPerSecond * flip;
		double rotorSpeed = Conversions.MPSToRPS(
				mPeriodicIO.targetVelocity, SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
		mPeriodicIO.driveDemand = new VoltageOut(rotorSpeed * SwerveConstants.kV)
				.withEnableFOC(true)
				.withOverrideBrakeDurNeutral(false);
	}

	public void setVelocity(SwerveModuleState desiredState) {
		double flip = setSteeringAngleOptimized(new Rotation2d(desiredState.angle.getRadians())) ? -1 : 1; //dc: modify to support WPILib Rotation2d constructor
		mPeriodicIO.targetVelocity = desiredState.speedMetersPerSecond * flip;
		double rotorSpeed = Conversions.MPSToRPS(
				mPeriodicIO.targetVelocity,
				Constants.SwerveConstants.wheelCircumference,
				Constants.SwerveConstants.driveGearRatio);

		if (Math.abs(rotorSpeed) < 0.002) {
			mPeriodicIO.driveDemand = new NeutralOut();
		} else {
			mPeriodicIO.driveDemand = new VelocityVoltage(rotorSpeed);
		}
	}

	/*DC.11.14.24. bugfix to turn wheels in right direction in teleop swerve mode
	* We need to negate the desired steering angle because position reading of our robot's rotation motor 
	* increases along clock-wise direction vs. CCW assumed in Kinematic.toSwerveModuleStates() to calculate disired moduleState.angle.
	* For the same reason, we need to negate setPosition() value in resetToAbsolute() too.
	*/
	private boolean setSteeringAngleOptimized(Rotation2d steerAngle) {
		boolean flip = false;
		final double targetClamped = - steerAngle.getDegrees();//See comments above for the negate operation
		final double angleUnclamped = getCurrentUnboundedDegrees();
		final Rotation2d angleClamped = Rotation2d.fromDegrees(angleUnclamped);
		final Rotation2d relativeAngle = Rotation2d.fromDegrees(targetClamped).rotateBy(angleClamped.unaryMinus()); //dc. replace citrus inverse() with wpilib unaryMinus()
		double relativeDegrees = relativeAngle.getDegrees();
 		if (relativeDegrees > 90.0) {
			relativeDegrees -= 180.0;
			flip = true;

		} else if (relativeDegrees < -90.0) {
			relativeDegrees += 180.0;
			flip = true;
		}
 		{//todo: debug code, TBR
//			if (mCounter++ >50){
//				mCounter =0;
//				SmartDashboard.putString("Module #"+ kModuleNumber+" setSteeringAngleOpti(), desiredAngle, angleUnclamped",
//					String.format("%.2f,%.2f", targetClamped, angleUnclamped));
//				SmartDashboard.putString("Module #"+ kModuleNumber+" setSteeringAngleOpti(), relativeDegreesPreFlip, relativeDegrees",
//					String.format("%.2f,%.2f", relativeDegreesPreFlip,relativeDegrees));
//			}
		}
		setSteeringAngleRaw(angleUnclamped + relativeDegrees);
		target_angle = angleUnclamped + relativeDegrees;
		return flip;
	}

	private double target_angle;

	private void setSteeringAngleRaw(double angleDegrees) {
		double rotorPosition = Conversions.degreesToRotation(angleDegrees, SwerveConstants.angleGearRatio);
//		mPeriodicIO.rotationDemand = new PositionDutyCycle(rotorPosition, 0.0, true, 0.0, 0, false, false, false);
		mPeriodicIO.rotationDemand = new PositionDutyCycle(rotorPosition);//dc: todo: check if the new constructor uses the same values for parameters deprecated from old version, as above
		SmartDashboard.putNumber("Drive/Module#" + kModuleNumber +"/AngleMotor/DemandAngle", angleDegrees);
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		mAngleMotor.setControl(mPeriodicIO.rotationDemand);
		mDriveMotor.setControl(mPeriodicIO.driveDemand);
	}

	/*DC.11.14.24. Bugfix for set wheel to straight forward position at teleop init positions
	* We need to negate setPosition() value because position reading of our robot's rotation motor 
	* increases along clock-wise direction while CCW assumed in original citrus code. 
	* So if current position is at the CW side of zero position, it takes a CCW movement (negative delta) 
	* for motor returns to zero position in setSteeringAngleOptimized; and vice versus. 
	*/
	public void resetToAbsolute() {
 		angleEncoder.getAbsolutePosition().waitForUpdate(Constants.kLongCANTimeoutMs);
		double angle = Util.placeInAppropriate0To360Scope(
				getCurrentUnboundedDegrees(), -(getCanCoder().getDegrees() - kAngleOffset)); //see above comments foor the negate operation
		double absolutePosition = Conversions.degreesToRotation(angle, SwerveConstants.angleGearRatio);
		//reset CANcoder reading to relative angle to Zero position, does NOT move motor
		Phoenix6Util.checkErrorAndRetry(() -> mAngleMotor.setPosition(absolutePosition, Constants.kLongCANTimeoutMs));
	}

/* TODO: TBR, keep the two test functions there for now in case we might need them to debug auto-mode
	public void straightenWheel() {
		angleEncoder.getAbsolutePosition().waitForUpdate(Constants.kLongCANTimeoutMs);
		double currAbsPosDegree = getCanCoder().getDegrees();
		double angle2Turn =  currAbsPosDegree - kAngleOffset;
		double motor2Rotate = angle2Turn /360 * SwerveConstants.angleGearRatio;
		double currRotorPos = mAngleMotor.getRotorPosition().getValueAsDouble();
		double newRotorPos = currRotorPos + motor2Rotate;
		System.out.println("Module#"+ kModuleNumber+ ".straightenModule(): currAbsPosDegree=" +currAbsPosDegree +",currRotorPos=" +currRotorPos + ", angle2Turn=" + angle2Turn + ", newRotorPos=" + newRotorPos);
		//mAngleMotor.setControl(new PositionDutyCycle(newRotorPos, 0.0, true, 0.0, 0, false, false, false));
		{//TODO: streamline calls used by loop in one function to verify behavior without the cycling. clean up after debug,
			mAngleMotor.setPosition(-motor2Rotate, Constants.kLongCANTimeoutMs);
			try{Thread.sleep(50);}catch(Exception e){}//need to wait 100ms for sensor signal to update back
			refreshSignals(); //update rotationPosition signal
			SmartDashboard.putString("Mod#"+kModuleNumber +" straightenWheel (curr, zero)",
					String.format("(%.2f,%.2f)", currAbsPosDegree, kAngleOffset));
//			SmartDashboard.putString("Mod#"+kModuleNumber +" rotatorPosition (before, after), and turn-wheel degree StraightenWheel().setPosition()",
//					String.format("(%.2f,%.2f,%.2f)", currRotorPos, mAngleMotor.getRotorPosition().getValueAsDouble(), angle2Turn));
//			SmartDashboard.putString("Mod#"+kModuleNumber +"StraightenWheel() mPeriodicIO.rotationPosition",
//					String.format("(%.5f)", mPeriodicIO.rotationPosition));
			Rotation2d targetDegree=Rotation2d.fromDegrees(0.0);
			setSteeringAngleOptimized(targetDegree);
			mAngleMotor.setControl(mPeriodicIO.rotationDemand);
		}
	}

	//TODO: debug swerve function, TBR
	public void swerveModule (Rotation2d swerveAngle){
		refreshSignals();
		SmartDashboard.putString("Mod#" + kModuleNumber + " swerveModule() (currAngle, swerveAngle)",
			String.format("%.2f,%.2f",getCurrentUnboundedDegrees(), swerveAngle.getDegrees()));
		setSteeringAngleOptimized(swerveAngle);
		mAngleMotor.setControl(mPeriodicIO.rotationDemand);
	}
*/
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
				Constants.SwerveConstants.wheelCircumference,
				Constants.SwerveConstants.driveGearRatio);
	}

	public double getDriveDistanceMeters() {
		return Conversions.rotationsToMeters(
				mPeriodicIO.drivePosition,
				Constants.SwerveConstants.wheelCircumference,
				Constants.SwerveConstants.driveGearRatio);
	}

	public double getCurrentUnboundedDegrees() {
		return Conversions.rotationsToDegrees(mPeriodicIO.rotationPosition, SwerveConstants.angleGearRatio);
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
		mDriveMotorSim.setInputVoltage(mDriveMotorSimState.getMotorVoltage());
		mDriveMotorSim.update(TimedRobot.kDefaultPeriod);

		double drivePosition = mDriveMotorSim.getAngularPositionRotations();
		double driveVelocity = mDriveMotorSim.getAngularVelocityRadPerSec() * Constants.Swerve.wheelCircumference / (2.0 * Math.PI);
		mDriveMotorSimState.setRawRotorPosition(drivePosition * Constants.Swerve.driveGearRatio);
		mDriveMotorSimState.setRotorVelocity(driveVelocity * Constants.Swerve.driveGearRatio);

		// Simulate steering
		mAngleMotorSim.setInputVoltage(mAngleMotorSimState.getMotorVoltage());
		mAngleMotorSim.update(TimedRobot.kDefaultPeriod);

		double steeringPosition = mAngleMotorSim.getAngularPositionRotations();
		double steeringVelocity = mAngleMotorSim.getAngularVelocityRadPerSec() * Constants.Swerve.wheelCircumference / (2.0 * Math.PI);
		mAngleMotorSimState.setRawRotorPosition(steeringPosition * Constants.Swerve.angleGearRatio);
		mAngleMotorSimState.setRotorVelocity(steeringVelocity * Constants.Swerve.angleGearRatio);

		angleEncoderSimState.setRawPosition(steeringPosition * Constants.Swerve.angleGearRatio);
		angleEncoderSimState.setVelocity(steeringVelocity * Constants.Swerve.angleGearRatio);
	}
}
