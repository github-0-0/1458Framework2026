package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.DigitalSensor;

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
  private TalonFX mRightMotor; //LEADER



  
  private TrapezoidProfile mProfile;
  private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
  private PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  private double prevUpdateTime = Timer.getFPGATimestamp();

  //private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);

  private void setUpElevatorMotor(TalonFX motor) {
    motor.getConfigurator().apply(Constants.Elevator.ElevatorConfiguration(),Constants.kLongCANTimeoutMs);
    
    // Set the motor to brake mode (will hold its position when powered off)
}
  private Elevator() {
    //super("Elevator");
	//TODO:figure out what this does

    mPeriodicIO = new PeriodicIO();

    // LEFT ELEVATOR MOTOR
    mLeftMotor = new TalonFX(Constants.Elevator.kElevatorLeftMotorId);
    // RIGHT ELEVATOR MOTOR
    mRightMotor = new TalonFX(Constants.Elevator.kElevatorRightMotorId);
    
    mLeftMotor.setControl(new Follower(mRightMotor.getDeviceID(), true));
    mLeftMotor.setNeutralMode(NeutralModeValue.Coast);
    mRightMotor.setNeutralMode(NeutralModeValue.Brake);

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

  public int currentState = 0;
  public int targetState = 0;

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
    goToTarget();

    // double curTime = Timer.getFPGATimestamp();
    // double dt = curTime - prevUpdateTime;
    // prevUpdateTime = curTime;
    // if (mPeriodicIO.is_elevator_pos_control) {
    //   // Update goal
    //   mGoalState.position = mPeriodicIO.elevator_target;

    //   // Calculate new state
    //   prevUpdateTime = curTime;
    //   mCurState = mProfile.calculate(dt, mCurState, mGoalState);

    //   // Set PID controller to new state
    //   /*mLeftPIDController.setReference(
    //       mCurState.position,
    //       CANSparkMax.ControlType.kPosition,
    //       0,
    //       Constants.Elevator.kG,
    //       ArbFFUnits.kVoltage);*/ //TODO: verify if this patch works
    //   mLeftMotor.setControl(m_request.withPosition(mCurState.position));
    // } else {
    //   //mCurState.position = mLeftEncoder.getPosition().getValueAsDouble();
    //   mCurState.velocity = 0;
    //   mLeftMotor.set(mPeriodicIO.elevator_power);
    
  }

  @Override
  public void stop() {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = 0.0;

    mRightMotor.set(0.0);
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

  /*
  public void reset() {
    //mLeftEncoder.setPosition(0.0);
  }
  */

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public ElevatorState getState() {
    return mPeriodicIO.state;
  }

  public ElevatorState getLevel() {
    if (DigitalSensor.getSensor(4)) {
      return ElevatorState.L4;
    } else if (DigitalSensor.getSensor(3)) {
      return ElevatorState.L3;
    } else if (DigitalSensor.getSensor(2)) {
      return ElevatorState.L2;
    } else if (DigitalSensor.getSensor(1)) {
      return ElevatorState.L1;
    } else if (DigitalSensor.getSensor(0)) {
      return ElevatorState.GROUND;
    } else {
      return ElevatorState.NONE;
    }
  }

public void setLevel() {
  for(int i = 0; i < 5; i++) {
    if(DigitalSensor.getSensor(i)) {
      currentState = i;
      break;
    }
  }
}



public void setTargetLevel(int target) {
  targetState = target;
  if(targetState > 4) {
    targetState = 0;
  }
  if(targetState < 0) {
    targetState = 4;
  }
}

public void incTarget() {
 setTargetLevel(targetState + 1);
}

public void decTarget() {
  setTargetLevel(targetState - 1);
 }

public void runElevator(double speed) {
  mRightMotor.set(speed);
}

public void goToTarget() {
  setLevel();
  if(targetState > 4 || targetState < 0) {
    targetState = 0;
  }
  if(Laser.inRangeIntake()) {
    return;
  }
  if(targetState == currentState) {
    stop();
    return;
  }
  else if(targetState > currentState) {
    runElevator(0.1);
    return;
  }
  else if(targetState < currentState) {
    runElevator(-0.1);
    return;
  }
}



  // public void setElevatorPower(double power) {
  //   SmartDashboard.putNumber("setElevatorPower", power);
  //   mPeriodicIO.is_elevator_pos_control = false;
  //   mPeriodicIO.elevator_power = power;
  // }

  // public void goToElevatorGround() {
  //   mPeriodicIO.is_elevator_pos_control = true;
  //   mPeriodicIO.elevator_target = Constants.Elevator.kGROUNDHeight;
  //   mPeriodicIO.state = ElevatorState.GROUND;
  // }
  
  // public void goToElevatorL1() {
  //   mPeriodicIO.is_elevator_pos_control = true;
  //   mPeriodicIO.elevator_target = Constants.Elevator.kL1Height;
  //   mPeriodicIO.state = ElevatorState.L1;
  // }

  // public void goToElevatorL2() {
  //   mPeriodicIO.is_elevator_pos_control = true;
  //   mPeriodicIO.elevator_target = Constants.Elevator.kL2Height;
  //   mPeriodicIO.state = ElevatorState.L2;
  // }

  // public void goToElevatorL3() {
  //   mPeriodicIO.is_elevator_pos_control = true;
  //   mPeriodicIO.elevator_target = Constants.Elevator.kL3Height;
  //   mPeriodicIO.state = ElevatorState.L3;
  // }

  // public void goToElevatorL4() {
  //   mPeriodicIO.is_elevator_pos_control = true;
  //   mPeriodicIO.elevator_target = Constants.Elevator.kL4Height;
  //   mPeriodicIO.state = ElevatorState.L4;
  // }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
