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

public class Hang extends Subsystem {
    private static Hang mInstance;
    private PeriodicIO mPeriodicIO = new PeriodicIO();

	public static Hang getInstance() {
		if (mInstance == null) {
		mInstance = new Hang();
		}
		return mInstance;
	}

    private class PeriodicIO {
        double speed = 0.0;
        HangState state = HangState.STOP;
    }

    private enum HangState {
        HANG,
        UNHANG,
        STOP,
        HOLD
    }

    private TalonFX mHangMotor;

    private Hang() {
        mHangMotor = new TalonFX(Constants.Hang.kHangMotorId);
        mHangMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}

            @Override
            public void onLoop(double timestamp) {
                switch (mPeriodicIO.state) {
                    case HANG:
                        hang();
                        break;
                    case UNHANG:
                        unhang();
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
            }
            
            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public void writePeriodicOutputs() {
        mHangMotor.set(mPeriodicIO.speed);
    }

    /*@Override
    public void outputTelemetry() {

    }

    @Override
    public void reset() {

    }*/

    public void hang() {
        mPeriodicIO.speed = Constants.Hang.kHangSpeed;
        mPeriodicIO.state = HangState.HANG;
    }

    public void unhang() {
        mPeriodicIO.speed = -Constants.Hang.kHangSpeed;
        mPeriodicIO.state = HangState.UNHANG;
    }

    public void stopHanger() {
        mPeriodicIO.state = HangState.STOP;
    }

    public void stop() {
        mPeriodicIO.speed = 0.0;
        mPeriodicIO.state = HangState.STOP;
    }

    public void hold() {
        mPeriodicIO.speed = Constants.Hang.kHoldSpeed;
        mPeriodicIO.state = HangState.HOLD;
    }
}
