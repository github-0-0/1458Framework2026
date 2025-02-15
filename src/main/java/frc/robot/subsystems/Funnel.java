package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
//import frc.robot.Helpers;
//import frc.robot.subsystems.leds.LEDs;
import frc.robot.Loops.ILooper;
import frc.robot.Loops.Loop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Funnel extends Subsystem {
	private static final double k_pivotMotorP = 0.12;
	private static final double k_pivotMotorI = 0.0;
	private static final double k_pivotMotorD = 0.001;

	private final PIDController m_pivotPID = new PIDController(k_pivotMotorP, k_pivotMotorI, k_pivotMotorD);

	private final CANcoder m_pivotEncoder = new CANcoder(99); //TODO: change ID


	//public final LEDs m_leds = LEDs.getInstance();

	/*-------------------------------- Private instance variables ---------------------------------*/
	private static Funnel mInstance;
	private PeriodicIO m_periodicIO;

	public static Funnel getInstance() {
		if (mInstance == null) {
		mInstance = new Funnel();
		}
		return mInstance;
	}

	private TalonFX mPivotMotor;

	private Funnel() {
		//super("Intake");
		mPivotMotor = new TalonFX(Constants.Funnel.kPivotMotorId/*  MotorType.kBrushless*/);
		mPivotMotor.setNeutralMode(NeutralModeValue.Brake);
		m_periodicIO = new PeriodicIO();
	}

	private static class PeriodicIO {
		// Input: Desired state
		PivotTarget pivot_target = PivotTarget.START;
		// Output: Motor set values
		double intake_pivot_voltage = 0.0;
	}

	public enum PivotTarget {
		START,
		END
	}

	/*-------------------------------- Generic Subsystem Functions --------------------------------*/

	@Override
		public void registerEnabledLoops(ILooper enabledLooper) {
			enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				double pivot_angle = pivotTargetToAngle(m_periodicIO.pivot_target);
				m_periodicIO.intake_pivot_voltage = m_pivotPID.calculate(getPivotAngleDegrees(), pivot_angle);
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}


	@Override
	public void writePeriodicOutputs() {
		mPivotMotor.setVoltage(m_periodicIO.intake_pivot_voltage);
	}

	@Override
	public void stop() {
		m_periodicIO.intake_pivot_voltage = 0.0;
	}

	@Override
	public void outputTelemetry() {
//		SmartDashboard.putNumber("Pivot/Abs Enc (getAbsolutePosition)", m_pivotEncoder.getAbsolutePosition().getValueAsDouble());	//dc.2.11.25, TODO: to be restored
		SmartDashboard.putNumber("Pivot/Abs Enc (getPivotAngleDegrees)", getPivotAngleDegrees());
		SmartDashboard.putNumber("Pivot/Setpoint", pivotTargetToAngle(m_periodicIO.pivot_target));

		SmartDashboard.putNumber("Pivot/Power", m_periodicIO.intake_pivot_voltage);
		SmartDashboard.putNumber("Pivot/Current (amps)", mPivotMotor.getTorqueCurrent().getValue().abs(Amps));
	}

	public double pivotTargetToAngle(PivotTarget target) {
 		switch (target) {
			case START:
				return Constants.Funnel.k_pivotStartAngle;
			case END:
				return Constants.Funnel.k_pivotEndAngle;
			default:
				// "Safe" default
				return 0.0;
			}
	}

	/*---------------------------------- Custom Public Functions ----------------------------------*/

	public double getPivotAngleDegrees() {
//		double value = m_pivotEncoder.getAbsolutePosition().getValueAsDouble() -
//			Constants.Funnel.k_pivotEncoderOffset + 0.5;
		return 0.;//Units.rotationsToDegrees(modRotations(value)); //dc.2.11.25, turn off cancoder to avoid CAM frame not receiving error; TODO: all cancoders shall go to Cancoders.java
	}

	// Pivot helper functions
	public void goToStart() {
		m_periodicIO.pivot_target = PivotTarget.START;
	}

	public void goToEnd() {
		m_periodicIO.pivot_target = PivotTarget.END;
	}

	public void setPivotTarget(PivotTarget target) {
		m_periodicIO.pivot_target = target;
	}

	public boolean isPivotAtTarget() {
		return Math.abs(getPivotAngleDegrees() - pivotTargetToAngle(m_periodicIO.pivot_target)) < 5;
	}

	/*---------------------------------- Custom Private Functions ---------------------------------*/

	public static double modRotations(double input) {
		input %= 1.0;
		if (input < 0.0) {
		input += 1.0;
		}
		return input;
	}

	public static double modDegrees(double input) {
		input %= 360.0;
		if (input < 0.0) {
		input += 360.0;
		}
		return input;
	}

	public static int clamp(int val, int min, int max) {
		return Math.max(min, Math.min(max, val));
	}
}
