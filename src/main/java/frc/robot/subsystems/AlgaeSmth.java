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



public class AlgaeSmth extends Subsystem {
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
        AlgaeState state = AlgaeState.STOP;
    }

    private enum AlgaeState {
        MAGTRIPPED,
        MAGNOTTRIPPED,
        RETRACT,
        PUSH,
        SPINPOSITIVE,
        SPINNEGATIVE,
        STOP
    }

    private TalonFX mAlgaeMotor;

    public AlgaeSmth() {
        mAlgaeMotor = new TalonFX(Constants.AlgaeSmth.kAlgaeSmthLeftMotorId);
        mAlgaeMotor = new TalonFX(Constants.AlgaeSmth.kAlgaeSmthRightMotorId);
        //mHangMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}

            @Override
            public void onLoop(double timestamp) {
                /*switch (mPeriodicIO.state) {
                    case StartAlgaeSmth:
                        AlgaeSmth();
                        break;
                    case StopAlgaeSmth:
                        stopAlgaeSmth();
                        break;
                    case HOLD:
                        hold();
                        break;
                    case STOP:
                        stop();
                        break;
                    default:
                        System.err.println("This should not be happening.");
                        break;
                }
                        */
            }
            
            @Override
            public void onStop(double timestamp) {
                //stop();
            }
        });
    }

    public void writePeriodicOutputs() {
        //mHangMotor.set(mPeriodicIO.speed);
    }

    /*@Override
    public void outputTelemetry() {

    }

    @Override
    public void reset() {

    }*/

    public void magSensorTripped() {
        mPeriodicIO.speed = Constants.Hang.kHangSpeed;
        mPeriodicIO.state = AlgaeState.MAGTRIPPED;
    }

    public void magSensorNotTripped() {
        mPeriodicIO.speed = -Constants.Hang.kHangSpeed;
        mPeriodicIO.state = AlgaeState.MAGTRIPPED;
    }

    public void AlgaeRetracted() {
        mPeriodicIO.state = AlgaeState.RETRACT;
    }

    public void AlgaePushed() {
        mPeriodicIO.speed = 0.0;
        mPeriodicIO.state = AlgaeState.PUSH;
    }

    public void AlgaePositive() {
        mPeriodicIO.speed = Constants.Hang.kHoldSpeed;
        mPeriodicIO.state = AlgaeState.SPINPOSITIVE;
    }
    
    public void AlgaeNegative() {
        mPeriodicIO.speed = -Constants.Hang.kHoldSpeed;
        mPeriodicIO.state = AlgaeState.SPINNEGATIVE;
    }

    public void setMotor(double speed) {
        mAlgaeMotor.set(speed);
    }


}
