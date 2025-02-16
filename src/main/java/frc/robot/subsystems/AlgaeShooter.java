package frc.robot.subsystems;

import java.util.zip.Checksum;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Loops.ILooper;
import frc.robot.Loops.Loop;
import frc.robot.subsystems.SwerveDrive.PeriodicIO;

public class AlgaeShooter extends Subsystem {

	/*-------------------------------- Private instance variables ---------------------------------*/
	private static AlgaeShooter mInstance;
	
	private PeriodicIO mPeriodicIO = new PeriodicIO();

	public static AlgaeShooter getInstance() {
		if (mInstance == null) {
		mInstance = new AlgaeShooter();
		}
		return mInstance;
	}

	private class PeriodicIO {
		double speed = 0.0;
		AlgaeShooterState state = AlgaeShooterState.STOP;
	} 
	
	private enum AlgaeShooterState {
		INTAKE,
		SHOOT,
		STOP
	}

	private TalonFX mLeftAlgaeShooterMotor;
	private TalonFX mRightAlgaeShooterMotor;

	private AlgaeShooter() {
		//super("AlgaeShooter");
		mLeftAlgaeShooterMotor = new TalonFX(Constants.AlgaeShooter.kAlgaeShooterLeftMotorId);
		mRightAlgaeShooterMotor = new TalonFX(Constants.AlgaeShooter.kAlgaeShooterRightMotorId); //LEADER
		mLeftAlgaeShooterMotor.setControl(new Follower(mRightAlgaeShooterMotor.getDeviceID(), true));
		mLeftAlgaeShooterMotor.setNeutralMode(NeutralModeValue.Brake);
        mRightAlgaeShooterMotor.setNeutralMode(NeutralModeValue.Brake);
	}

	/*-------------------------------- Generic Subsystem Functions --------------------------------*/

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				switch (mPeriodicIO.state) {
					case INTAKE:
						if (!Laser.inRangeAlgaeShooter()) {
							spinIn();
						} else {
							stop();
						}
						break;
					case SHOOT:
						spinOut();
						//if(Laser.getMeasurementAlgaeShooter() > 300) {
						//	stop();
						//}	
					spinOut();
						break;
					case STOP:
						stop();
						break;
					default:
						System.err.println("How did this happen?");
						break;
				}
			}

			@Override
			public void onStop(double timestamp) {
				stop();
			}
		});
	}

	@Override
	public void writePeriodicOutputs() {
		mRightAlgaeShooterMotor.set(mPeriodicIO.speed);
	}

	@Override
	public void outputTelemetry() {

	}

	/*
	@Override
	public void reset() {
	}
	*/

	/*---------------------------------- Custom Public Functions ----------------------------------*/

	public void intake() {
		mPeriodicIO.state = AlgaeShooterState.INTAKE;
	}

	public void shoot() {
		mPeriodicIO.state = AlgaeShooterState.SHOOT;
	}

	public void stopAlgaeShooter() {
		mPeriodicIO.state = AlgaeShooterState.STOP;
	}
	/*---------------------------------- Custom Private Functions ---------------------------------*/

	public void spinOut() {
		mPeriodicIO.speed = Constants.AlgaeShooter.kAlgaeShooterSpeed;
	}

    public void spinIn() {
        mPeriodicIO.speed = -Constants.AlgaeShooter.kAlgaeShooterSpeed;
    }
	
	@Override
	public void stop() {
		mPeriodicIO.speed = 0.0;
		mPeriodicIO.state = AlgaeShooterState.STOP;
	}
}
