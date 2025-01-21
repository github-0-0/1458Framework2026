package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
//written with chatgpt (very useful)
public class Elevator extends Subsystem {

    private static Elevator mInstance;
    private PeriodicIO mPeriodicIO;
    private TalonFX mLeftMotor;
    private TalonFX mRightMotor;
    private DigitalInput magneticLimitSensor = new DigitalInput(0);
    private int currentLevelIndex = 0;
    private boolean lastMagnetState = false;
    private boolean movingUp = false;
    private TrapezoidProfile mProfile;
    private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
    private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
    private PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    private double prevUpdateTime = Timer.getFPGATimestamp();

    public static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    private Elevator() {
        mPeriodicIO = new PeriodicIO();

        mLeftMotor = new TalonFX(Constants.Elevator.kElevatorLeftMotorId);
        setUpElevatorMotor(mLeftMotor);

        mRightMotor = new TalonFX(Constants.Elevator.kElevatorRightMotorId);
        setUpElevatorMotor(mRightMotor);
        mRightMotor.setControl(new DutyCycleOut(mLeftMotor.getDutyCycle().getValue()));

        mProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAcceleration)
        );
    }

    private void setUpElevatorMotor(TalonFX motor) {
        motor.getConfigurator().apply(Constants.Elevator.ElevatorConfiguration(), Constants.kLongCANTimeoutMs);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public enum ElevatorState {
        NONE, GROUND, L1, L2, L3, L4
    }

    private static class PeriodicIO {
        double elevator_target = 0.0;
        double elevator_power = 0.0;
        boolean is_elevator_pos_control = false;
        ElevatorState state = ElevatorState.GROUND;
    }

    @Override
    public void writePeriodicOutputs() {
        double curTime = Timer.getFPGATimestamp();
        double dt = curTime - prevUpdateTime;
        prevUpdateTime = curTime;

        if (mPeriodicIO.is_elevator_pos_control) {
            mGoalState.position = mPeriodicIO.elevator_target;
            mCurState = mProfile.calculate(dt, mCurState, mGoalState);
            mLeftMotor.setControl(m_request.withPosition(mCurState.position));
        } else {
            mCurState.velocity = 0;
            mLeftMotor.set(mPeriodicIO.elevator_power);
        }

        movingUp = mPeriodicIO.elevator_power > 0;
        updateLevel();
        mLeftMotor.set(mPeriodicIO.elevator_power);
    }

    @Override
    public void stop() {
        mPeriodicIO.is_elevator_pos_control = false;
        mPeriodicIO.elevator_power = 0.0;
        mLeftMotor.set(0.0);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Position/Target", mPeriodicIO.elevator_target);
        SmartDashboard.putNumber("Position/Setpoint", mCurState.position);
        SmartDashboard.putNumber("Velocity/Setpoint", mCurState.velocity);
        SmartDashboard.putNumber("Current/Left", mLeftMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Current/Right", mRightMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Magnet Detected", magneticLimitSensor.get());
        SmartDashboard.putNumber("Current Level Index", currentLevelIndex);
        SmartDashboard.putString("Current Level State", getLevel().toString());
    }

    private void updateLevel() {
        boolean magnetDetected = magneticLimitSensor.get();
        if (magnetDetected && !lastMagnetState) {
            if (movingUp && currentLevelIndex < 4) {
                currentLevelIndex++;
            } else if (!movingUp && currentLevelIndex > 0) {
                currentLevelIndex--;
            }
        }
        lastMagnetState = magnetDetected;
    }

    public ElevatorState getLevel() {
        switch (currentLevelIndex) {
            case 0: return ElevatorState.GROUND;
            case 1: return ElevatorState.L1;
            case 2: return ElevatorState.L2;
            case 3: return ElevatorState.L3;
            case 4: return ElevatorState.L4;
            default: return ElevatorState.NONE;
        }
    }

    public void setElevatorPower(double power) {
        mPeriodicIO.is_elevator_pos_control = false;
        mPeriodicIO.elevator_power = power;
    }

    public void goToElevatorGround() {
        setElevatorTarget(Constants.Elevator.kGROUNDHeight, ElevatorState.GROUND);
    }

    public void goToElevatorL1() {
        setElevatorTarget(Constants.Elevator.kL1Height, ElevatorState.L1);
    }

    public void goToElevatorL2() {
        setElevatorTarget(Constants.Elevator.kL2Height, ElevatorState.L2);
    }

    public void goToElevatorL3() {
        setElevatorTarget(Constants.Elevator.kL3Height, ElevatorState.L3);
    }

    public void goToElevatorL4() {
        setElevatorTarget(Constants.Elevator.kL4Height, ElevatorState.L4);
    }

    private void setElevatorTarget(double target, ElevatorState state) {
        mPeriodicIO.is_elevator_pos_control = true;
        mPeriodicIO.elevator_target = target;
        mPeriodicIO.state = state;
    }
}
