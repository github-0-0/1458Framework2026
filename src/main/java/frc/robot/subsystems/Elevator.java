package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Elevator mInstance;
  private PeriodicIO mPeriodicIO;

  // private static final double kPivotCLRampRate = 0.5;
  // private static final double kCLRampRate = 0.5;

  public static Elevator getInstance() {
    if (mInstance == null) {
      mInstance = new Elevator();
    }
    return mInstance;
  }

  private TalonFX mLeftMotor;
  //private CANcoder mLeftEncoder;
  DigitalInput level0 = new DigitalInput(0);
  DigitalInput level1 = new DigitalInput(1);
  DigitalInput level2 = new DigitalInput(2);
  DigitalInput level3 = new DigitalInput(3);
  DigitalInput level4 = new DigitalInput(4);

  private TalonFX mRightMotor;
  //private CANcoder mRightEncoder;

  private TrapezoidProfile mProfile;
  private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
  private PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  private double prevUpdateTime = Timer.getFPGATimestamp();

  // private RelativeEncoder mLeftEncoder;
  // private SparkMaxLimitSwitch mLowerLimit;
  // private SparkMaxLimitSwitch mUpperLimit;

  // private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);

  private void setUpElevatorMotor(TalonFX motor) {
    motor.getConfigurator().apply(Constants.Elevator.ElevatorConfiguration(),Constants.kLongCANTimeoutMs);
    
    // Set the motor to brake mode (will hold its position when powered off)
    motor.setNeutralMode(NeutralModeValue.Brake);
}
  private Elevator() {
    //super("Elevator");
	//TODO:figure out what this does

    mPeriodicIO = new PeriodicIO();

    // LEFT ELEVATOR MOTOR
    mLeftMotor = new TalonFX(Constants.Elevator.kElevatorLeftMotorId);
    //mLeftEncoder = Cancoders.getInstance().getElevatorLeft();
    setUpElevatorMotor(mLeftMotor);

    // RIGHT ELEVATOR MOTOR
    mRightMotor = new TalonFX(Constants.Elevator.kElevatorRightMotorId);
    //mRightEncoder = Cancoders.getInstance().getElevatorRight();
    setUpElevatorMotor(mRightMotor);

    // mRightMotor.setInverted(true);
    mRightMotor.setControl(new DutyCycleOut(mLeftMotor.getDutyCycle().getValue()));

    //mLeftMotor.burnFlash();
    //mRightMotor.burnFlash();
    //TODO: figure out burnflash equivalent
    mProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAcceleration));
  }

  public enum ElevatorState {
    NONE,
    GROUND,
    L1,
    L2,
    L3,
    L4,
  }

  private static class PeriodicIO {
    double elevator_target = 0.0;
    double elevator_power = 0.0;

    boolean is_elevator_pos_control = false;

    ElevatorState state = ElevatorState.GROUND;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  //@Override
  //public void periodic() {
    // TODO: Use this pattern to only drive slowly when we're really high up
    // if(mPivotEncoder.getPosition() > Constants.kPivotScoreCount) {
    // mPeriodicIO.is_pivot_low = true;
    // } else {
    // mPeriodicIO.is_pivot_low = false;
    // }
  //}

  @Override
  public void writePeriodicOutputs() {
    double curTime = Timer.getFPGATimestamp();
    double dt = curTime - prevUpdateTime;
    prevUpdateTime = curTime;
    if (mPeriodicIO.is_elevator_pos_control) {
      // Update goal
      mGoalState.position = mPeriodicIO.elevator_target;

      // Calculate new state
      prevUpdateTime = curTime;
      mCurState = mProfile.calculate(dt, mCurState, mGoalState);

      // Set PID controller to new state
      /*mLeftPIDController.setReference(
          mCurState.position,
          CANSparkMax.ControlType.kPosition,
          0,
          Constants.Elevator.kG,
          ArbFFUnits.kVoltage);*/ //TODO: verify if this patch works
      mLeftMotor.setControl(m_request.withPosition(mCurState.position));
    } else {
      //mCurState.position = mLeftEncoder.getPosition().getValueAsDouble();
      mCurState.velocity = 0;
      mLeftMotor.set(mPeriodicIO.elevator_power);
    }
  }

  @Override
  public void stop() {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = 0.0;

    mLeftMotor.set(0.0);
  }

  @Override
  public void outputTelemetry() {
    //SmartDashboard.putNumber("Position/Current", mLeftEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Position/Target", mPeriodicIO.elevator_target);
    //SmartDashboard.putNumber("Velocity/Current", mLeftEncoder.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("Position/Setpoint", mCurState.position);
    SmartDashboard.putNumber("Velocity/Setpoint", mCurState.velocity);

    SmartDashboard.putNumber("Current/Left", mLeftMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Current/Right", mRightMotor.getSupplyCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Output/Left", mLeftMotor.getMotorOutputStatus().getValueAsDouble());
    SmartDashboard.putNumber("Output/Right", mRightMotor.getMotorOutputStatus().getValueAsDouble());

    //SmartDashboard.putNumber("State", mPeriodicIO.state);
	//TODO: figure out how to put a state in smart dashbaord
}


  public void reset() {
    //mLeftEncoder.setPosition(0.0);
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public ElevatorState getState() {
    return mPeriodicIO.state;
  }

  public ElevatorState getLevel() {
    if (level4.get()) {
      return ElevatorState.L4;
    } else if (level3.get()) {
      return ElevatorState.L3;
    } else if (level2.get()) {
      return ElevatorState.L2;
    } else if (level1.get()) {
      return ElevatorState.L1;
    } else if (level0.get()) {
      return ElevatorState.GROUND;
    } else {
      return ElevatorState.NONE;
    }
  }

  public void setElevatorPower(double power) {
    SmartDashboard.putNumber("setElevatorPower", power);
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = power;
  }

  public void goToElevatorGround() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kGROUNDHeight;
    mPeriodicIO.state = ElevatorState.GROUND;
  }
  
  public void goToElevatorL1() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kL1Height;
    mPeriodicIO.state = ElevatorState.L1;
  }

  public void goToElevatorL2() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kL2Height;
    mPeriodicIO.state = ElevatorState.L2;
  }

  public void goToElevatorL3() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kL3Height;
    mPeriodicIO.state = ElevatorState.L3;
  }

  public void goToElevatorL4() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kL4Height;
    mPeriodicIO.state = ElevatorState.L4;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
