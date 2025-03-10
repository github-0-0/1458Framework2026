package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.lib.drivers.Phoenix6Util;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
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
  private TalonFX mRightMotor; // LEADER

  private MotionMagicVoltage m_request;
  private double prevUpdateTime = Timer.getFPGATimestamp();

  // private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);

  private void setUpElevatorMotor(TalonFX motor) {
    motor.getConfigurator().apply(Constants.Elevator.ElevatorConfiguration(), Constants.kLongCANTimeoutMs);

    // Set the motor to brake mode (will hold its position when powered off)
  }

  private Elevator() {
    // super("Elevator");
    // TODO:figure out what this does

    mPeriodicIO = new PeriodicIO();

    // LEFT ELEVATOR MOTOR
    mLeftMotor = new TalonFX(Constants.Elevator.kElevatorLeftMotorId); // MASTER
    // RIGHT ELEVATOR MOTOR
    mRightMotor = new TalonFX(Constants.Elevator.kElevatorRightMotorId);

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.0; // Add 0.0 V output to overcome static friction
    slot0Configs.kV = 0.0; // A velocity target of 1 rps results in 0.0 V output
    slot0Configs.kP = 0.8; // An error of 1 rotation results in 0.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.0; // A velocity of 1 rps results in 0.0 V output

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 240; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600;

    mRightMotor.getConfigurator().apply(talonFXConfigs);
    mLeftMotor.getConfigurator().apply(talonFXConfigs);

    mRightMotor.setControl(new Follower(mLeftMotor.getDeviceID(), true));
    mLeftMotor.setNeutralMode(NeutralModeValue.Coast);
    mLeftMotor.setNeutralMode(NeutralModeValue.Brake);

    mLeftMotor.setControl(new DutyCycleOut(mLeftMotor.getDutyCycle().getValue()));

    m_request = new MotionMagicVoltage(0);
  }

  public enum ElevatorState {
    NONE,
    GROUND,
    L1,
    L2,
    L3,
    L4,
  }

  private int currentState = 0;
  private int targetState = 0;
  private double targetRot = 0;

  private static class PeriodicIO {
    double elevator_target = 0.0;
    double elevator_power = 0.0;

    boolean is_elevator_pos_control = false;

    ElevatorState state = ElevatorState.GROUND;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  // @Override
  // public void periodic() {
  // TODO: Use this pattern to only drive slowly when we're really high up
  // if(mPivotEncoder.getPosition() > Constants.kPivotScoreCount) {
  // mPeriodicIO.is_pivot_low = true;
  // } else {
  // mPeriodicIO.is_pivot_low = false;
  // }
  // }

  @Override
  public void writePeriodicOutputs() {
    // goToTarget();
  }

  @Override
  public void stop() {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = 0.0;

    mLeftMotor.setVoltage(0);
  }

  @Override
  public void outputTelemetry() {
    // SmartDashboard.putNumber("Position/Current",
    // mLeftEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Position/Target", mPeriodicIO.elevator_target);
    // SmartDashboard.putNumber("Velocity/Current",
    // mLeftEncoder.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("Current/Left", mLeftMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Current/Right", mRightMotor.getSupplyCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Output/Left", mLeftMotor.getMotorOutputStatus().getValueAsDouble());
    SmartDashboard.putNumber("Output/Right", mRightMotor.getMotorOutputStatus().getValueAsDouble());

    // SmartDashboard.putNumber("State", mPeriodicIO.state);
    // TODO: figure out how to put a state in smart dashbaord
  }

  /*
   * public void reset() {
   * //mLeftEncoder.setPosition(0.0);
   * }
   */

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public ElevatorState getState() {
    return mPeriodicIO.state;
  }

  // public ElevatorState getLevel() {
  // if (DigitalSensor.getSensor(4)) {
  // return ElevatorState.L4;
  // } else if (DigitalSensor.getSensor(3)) {
  // return ElevatorState.L3;
  // } else if (DigitalSensor.getSensor(2)) {
  // return ElevatorState.L2;
  // } else if (DigitalSensor.getSensor(1)) {
  // return ElevatorState.L1;
  // } else if (DigitalSensor.getSensor(0)) {
  // return ElevatorState.GROUND;
  // } else {
  // return ElevatorState.NONE;
  // }
  // }

  public void updateLocation() {
    for (int i = 0; i < 4; i++) {
      if (DigitalSensor.getSensor(i)) {
        currentState = i;
        break;
      }
    }
  }

  public void setTargetLevel(int target) {
    targetState = target;
    if (targetState > 3) {
      targetState = 3;
    }
    if (targetState < 0) {
      targetState = 0;
    }
    switch (targetState) {
      case 0:
        targetRot = 0.1;
        break;
      case 1:
        targetRot = 11.229;
        break;
      case 2:
        targetRot = 23.44;
        break;
      case 3:
        targetRot = 45;
        break;
      default:
        targetRot = 0.1;
        break;

    }
  }

  public int getTarget() {
    return targetState;
  }

  public int getCurr() {
    return currentState;
  }

  public double getTargRot() {
    return targetRot;
  }

  public double getRot() {
    return mLeftMotor.getPosition().getValueAsDouble();
  }

  public void resetRot() {
    mRightMotor.setPosition(0);
  }

  public void incTarget() {
    setTargetLevel(targetState + 1);
  }

  public void decTarget() {
    setTargetLevel(targetState - 1);
  }

  public void runElevator(double speed) {
    mLeftMotor.set(speed);
  }

  public boolean goToTarget() {
    double currentRot = getRot();

    if (Laser.inRangeIntake()) {
      System.out.println("Break Laser Check");
      return false;
    }
    mLeftMotor.setControl(m_request.withPosition(targetRot));

    if (Math.abs(Math.abs(currentRot) - Math.abs(targetRot)) < 0.5) {
      System.out.println("At Location");
      return true;
    } else if (targetRot < currentRot) {
      System.out.println("Moving Down");
      return false;
    } else if (targetRot > currentRot) {
      System.out.println("Moving Up");
      return false;
    }
    System.out.println("No Case");
    return false;
  }

  // public void setElevatorPower(double power) {
  // SmartDashboard.putNumber("setElevatorPower", power);
  // mPeriodicIO.is_elevator_pos_control = false;
  // mPeriodicIO.elevator_power = power;
  // }

  // public void goToElevatorGround() {
  // mPeriodicIO.is_elevator_pos_control = true;
  // mPeriodicIO.elevator_target = Constants.Elevator.kGROUNDHeight;
  // mPeriodicIO.state = ElevatorState.GROUND;
  // }

  // public void goToElevatorL1() {
  // mPeriodicIO.is_elevator_pos_control = true;
  // mPeriodicIO.elevator_target = Constants.Elevator.kL1Height;
  // mPeriodicIO.state = ElevatorState.L1;
  // }

  // public void goToElevatorL2() {
  // mPeriodicIO.is_elevator_pos_control = true;
  // mPeriodicIO.elevator_target = Constants.Elevator.kL2Height;
  // mPeriodicIO.state = ElevatorState.L2;
  // }

  // public void goToElevatorL3() {
  // mPeriodicIO.is_elevator_pos_control = true;
  // mPeriodicIO.elevator_target = Constants.Elevator.kL3Height;
  // mPeriodicIO.state = ElevatorState.L3;
  // }

  // public void goToElevatorL4() {
  // mPeriodicIO.is_elevator_pos_control = true;
  // mPeriodicIO.elevator_target = Constants.Elevator.kL4Height;
  // mPeriodicIO.state = ElevatorState.L4;
  // }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}