package frc.robot.lib.drivers;

import java.util.Optional;

//dc.10.21.2024 ported from com.team1678.lib.drivers;
//replace citrus Rotation2d class with wpi version,
//???therefore, we use unaryMinus() call to replaces inverse() call in original citrus code
//TODO: check if pigeon gyro is installed inverted on my robot
//TODO: verify wpilib Rotation2d behaves the same as citrus code

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import frc.robot.Constants;
import frc.robot.Ports;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.units.Units.*;


public class Pigeon {

	private static Pigeon mInstance;

	public static Pigeon getInstance() {
		if (mInstance == null) {
			mInstance = new Pigeon(Ports.PIGEON);
		}
		return mInstance;
	}

	// Actual pigeon object
	private final Pigeon2 mGyro;

	// Configs
	private boolean inverted = Constants.SwerveConstants.invertGyro;
	private Rotation2d yawAdjustmentAngle = new Rotation2d();
	private Rotation2d rollAdjustmentAngle = new Rotation2d();
	private Rotation2d pitchAdjustmentAngle = new Rotation2d();

	private Pigeon(int port) {
		mGyro = new Pigeon2(port, "CV"); //TODO: ADD TO CONSTANTS
		mGyro.getConfigurator().apply(new Pigeon2Configuration());
		//todo: move to hardcoded yawAdjustmentAngle to robot init . 
		Optional<Alliance> ally = DriverStation.getAlliance();
		if (ally.isPresent() && ally.get() == Alliance.Blue) {
			System.out.println("Set rotation 180");
			yawAdjustmentAngle = Rotation2d.fromDegrees(180);
		}
	}

	public Rotation2d getYaw() {
		Rotation2d angle = getUnadjustedYaw().rotateBy(yawAdjustmentAngle.unaryMinus());
		if (inverted) {
			return angle.unaryMinus();
		}
		return angle;
	}

	public Rotation2d getRoll() {
		return getUnadjustedRoll().rotateBy(rollAdjustmentAngle.unaryMinus());
	}

	public Rotation2d getPitch() {
		return getUnadjustedPitch().rotateBy(pitchAdjustmentAngle.unaryMinus()).unaryMinus();
	}

	/**
	 * Sets the yaw register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setYaw(double angleDeg) {
		yawAdjustmentAngle = Rotation2d.fromDegrees(getYawStatusSignal().getValueAsDouble())
				.rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
	}

	/**
	 * Sets the roll register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setRoll(double angleDeg) {
		rollAdjustmentAngle =
				getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
	}

	/**
	 * Sets the roll register to read the specified value.
	 *
	 * @param angleDeg New yaw in degrees
	 */
	public void setPitch(double angleDeg) {
		pitchAdjustmentAngle =
				getUnadjustedPitch().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
		System.out.println("Reset gyro to " + getPitch().getDegrees());
	}

	public Rotation2d getUnadjustedYaw() {		
		return Rotation2d.fromDegrees(
				BaseStatusSignal.getLatencyCompensatedValueAsDouble(getYawStatusSignal(), getRateStatusSignal()));// dc: updates to 2025 API
	}

	public Rotation2d getUnadjustedPitch() {
		return Rotation2d.fromDegrees(mGyro.getRoll().getValueAsDouble());
	}

	public Rotation2d getUnadjustedRoll() {
		return Rotation2d.fromDegrees(mGyro.getPitch().getValueAsDouble());
	}

	public StatusSignal<Angle> getYawStatusSignal() {
		return mGyro.getYaw(); // dc. update to 2025 API, return measure in degree
	}

	public StatusSignal<AngularVelocity> getRateStatusSignal() {
		return mGyro.getAngularVelocityZDevice(); // dc. update to 2025 API, return measure in DPS (degree per second)
	}

	//dc.2.21.25, bugfix for pigeon simulation
	// new angle shall be based on adjustedYaw.
	public void updateSimPeriodic(double angularVelocity) {
		Pigeon2SimState gyroSimState = mGyro.getSimState();

		gyroSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

		Rotation2d angleChange = Rotation2d.fromRadians(angularVelocity * TimedRobot.kDefaultPeriod);
		Rotation2d angle = getUnadjustedYaw().plus(angleChange);
		gyroSimState.setRawYaw(angle.getDegrees());
	}
}
