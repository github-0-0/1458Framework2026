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
import edu.wpi.first.math.util.Units;
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
import frc.robot.Robot;
import frc.robot.Loops.ILooper;
import frc.robot.Loops.Loop;
import frc.robot.lib.swerve.SwerveModule;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.DigitalSensor;

public class Elevator extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Elevator mInstance=null;;
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

  // Simulation classes help us simulate the elevator.
  // Create an ElevatorSim to model the elevator
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
        LinearSystemId.createElevatorSystem(
          DCMotor.getKrakenX60Foc(2), 5.0, Units.inchesToMeters(5.0), 20.0),
        DCMotor.getKrakenX60Foc(2),
        0.5,
        5.0,
        true,
        1.0);

  private final double elevatorTravelPerRotation = 0.065;

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d mech = new Mechanism2d(1.0, 1.0);
  private final MechanismRoot2d elevator = mech.getRoot("Elevator", 0.5, 0);
  private final MechanismLigament2d elevatorViz =
      elevator.append(
          new MechanismLigament2d(
            "Shaft", 0.5, 90, 25.0, new Color8Bit(Color.kYellow)
        )
      );

  private MotionMagicVoltage m_request;
  private boolean mSafeStop = true;


  private Elevator() {

    mPeriodicIO = new PeriodicIO();

    // LEFT ELEVATOR MOTOR
    mLeftMotor = new TalonFX(Constants.Elevator.kElevatorLeftMotorId); // MASTER
    // RIGHT ELEVATOR MOTOR
    mRightMotor = new TalonFX(Constants.Elevator.kElevatorRightMotorId);

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = Constants.Elevator.kS; // Add 0.0 V output to overcome static friction
    slot0Configs.kV = Constants.Elevator.kV; // A velocity target of 1 rps results in 0.0 V output
    slot0Configs.kP = Constants.Elevator.kP; // An error of 1 rotation results in 0.4 V output
    slot0Configs.kI = Constants.Elevator.kI; // no output for integrated error
    slot0Configs.kD = Constants.Elevator.kD; // A velocity of 1 rps results in 0.0 V output

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Elevator.kCruiseVelocity; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = Constants.Elevator.kAcceleration; // Target acceleration of 240 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = Constants.Elevator.kJerk;

    mRightMotor.getConfigurator().apply(talonFXConfigs);
    mLeftMotor.getConfigurator().apply(talonFXConfigs);

    mRightMotor.setControl(new Follower(mLeftMotor.getDeviceID(), true));
    mLeftMotor.setNeutralMode(NeutralModeValue.Coast);
    mLeftMotor.setNeutralMode(NeutralModeValue.Brake);

    mLeftMotor.setControl(new DutyCycleOut(mLeftMotor.getDutyCycle().getValue()));

    m_request = new MotionMagicVoltage(0);

    SmartDashboard.putData("Elevator", mech);
  }

  public enum ElevatorState {
    GROUND,
    L2,
    L3,
    L4,
    AP,
    A1,
    A2,
    DEF,
  }

  private static class PeriodicIO {
    double elevator_target = 0.0;
    String state = "Ground";
    double mCurrentPos =0.0;//current encoder reading
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

  	/*-------------------------------- Generic Subsystem Functions --------------------------------*/

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
        mSafeStop=true;
        System.out.println("safestop = true;");
      }

			@Override
			public void onLoop(double timestamp) {

			}

			@Override
			public void onStop(double timestamp) {
				stop();
			}
		});
	}

  @Override
  public void readPeriodicInputs() {
    mPeriodicIO.mCurrentPos = mLeftMotor.getPosition().getValueAsDouble();//update elevator current position
  }


  @Override
  public void writePeriodicOutputs() {
    if (!isAtTarget() && !mSafeStop){
      goToTarget();
    }else{
      //System.out.println("it is at target =" );
      //System.out.println("it is at target =" );
      runElevatorRaw(0.03);
    }

    if (Robot.isSimulation()) {
      updateSimPeriodic();
    }
  }

  @Override
  public void stop() {
    mPeriodicIO.elevator_target = mLeftMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Position/Target", mPeriodicIO.elevator_target);
    SmartDashboard.putString("State", mPeriodicIO.state);
  }

  public void resetRot(double pos) {
    mRightMotor.setPosition(pos);
  }


  public void runElevatorRaw(double speed) {
    mLeftMotor.set(speed);
  }

  public synchronized void setTarget(String targ) {
    mSafeStop=false;
    switch(targ) {
      case "Ground":
        mPeriodicIO.elevator_target = Constants.Elevator.kGroundHeight;
        mPeriodicIO.state = "Ground";
        break;
      case "L2":
        mPeriodicIO.elevator_target = Constants.Elevator.kL2Height;
        mPeriodicIO.state = "L2";
        break;
      case "L3":
        mPeriodicIO.elevator_target = Constants.Elevator.kL3Height;
        mPeriodicIO.state = "L3";
        break;
      case "L4":
        mPeriodicIO.elevator_target = Constants.Elevator.kL4Height;
        mPeriodicIO.state = "L4";
        break;
      case "AP":
        mPeriodicIO.elevator_target = Constants.Elevator.kAPHeight;
        mPeriodicIO.state = "AP";
        break;
      case "A1":
        mPeriodicIO.elevator_target = Constants.Elevator.kA1Height;
        mPeriodicIO.state = "A1";
      case "A2":
        mPeriodicIO.elevator_target = Constants.Elevator.kA2Height;
        mPeriodicIO.state = "A2";
      case "DEF":
        mPeriodicIO.elevator_target = Constants.Elevator.KDefaultHeight;
        mPeriodicIO.state = "DEF";
    }
  }


  private void goToTarget() {

    if (Laser.inRangeIntake()) {
      //System.out.println("Break Laser Check");
      return;
    }
    //System.out.println("Elevator: Going to Target: " + mPeriodicIO.elevator_target);
    //System.out.println("Elevator: Going to Target: " + mPeriodicIO.elevator_target);
    mLeftMotor.setControl(m_request.withPosition(mPeriodicIO.elevator_target));


  }

  //dc.10.21.25, bugfix, caller usually is from another thread, direct access to elevator motors shall be avoid in this function
  // instead read the states inside readPeriodicInputs and store them for use here.
  //
  public synchronized boolean isAtTarget() {
    //System.out.println("reading");
    //System.out.println("Current Pos: " + mPeriodicIO.mCurrentPos);
    //System.out.println("Error: " + (mPeriodicIO.mCurrentPos - mPeriodicIO.elevator_target));

    return Math.abs(mPeriodicIO.mCurrentPos - mPeriodicIO.elevator_target) < 0.5;
  }

  // /** Advance the simulation of the elevator. */
  public void updateSimPeriodic() {

    // In this method, we update our simulation of what our elevator is doing
    TalonFXSimState mLeftMotorSim = mLeftMotor.getSimState();
    mLeftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // First, we set our "inputs" (voltages)
    elevatorSim.setInputVoltage(mLeftMotorSim.getMotorVoltage());

    // Next, we update it for the standard loop time
    elevatorSim.update(TimedRobot.kDefaultPeriod);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    // Convert elevator position to rotations for the motor
    mLeftMotorSim.setRawRotorPosition(elevatorSim.getPositionMeters() / elevatorTravelPerRotation);
    mLeftMotorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    elevatorViz.setLength(elevatorViz.getLength()
      + elevatorSim.getVelocityMetersPerSecond() * TimedRobot.kDefaultPeriod);
  }
}