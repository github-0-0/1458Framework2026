package frc.robot.subsystems;

import java.time.Period;
import java.util.zip.Checksum;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
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
import frc.robot.subsystems.SwerveDrive.PeriodicIO;
//both shooter and intake for coral
public class CoralShooter extends Subsystem {

	/*-------------------------------- Private instance variables ---------------------------------*/
	private static CoralShooter mInstance;

	private PeriodicIO mPeriodicIO = new PeriodicIO();

	public static CoralShooter getInstance() {
		if (mInstance == null) {
		mInstance = new CoralShooter();
		}
		return mInstance;
	}

	private class PeriodicIO {
		double speed = 0.0;
		ShooterState state = ShooterState.INTAKE;
		boolean isShooting = false;
	}

	private enum ShooterState {
		INTAKE,
		SHOOT,
		STOP
	}

	private TalonFX mLeftShooterMotor;
	private TalonFX mRightShooterMotor;

	// Simulation classes help us simulate the shooter.
	private final LinearSystemSim<N1, N1, N1> shooterSim =
		new LinearSystemSim<N1, N1, N1>(LinearSystemId.identifyVelocitySystem(25, 25));

	// Create a Mechanism2d visualization of the shooter
	private final Mechanism2d mech = new Mechanism2d(1.0, 1.0);
	private final MechanismRoot2d shooter = mech.getRoot("Shooter", 0.5, 0.5);
	private final MechanismLigament2d shooterViz =
		shooter.append(
			new MechanismLigament2d(
				"Shaft", 0.05, 0, 10.0, new Color8Bit(Color.kBlue)
			)
		);

	private CoralShooter() {
		//super("Shooter");
		mLeftShooterMotor = new TalonFX(Constants.CoralShooter.kShooterLeftMotorId);
		mRightShooterMotor = new TalonFX(Constants.CoralShooter.kShooterRightMotorId); //LEADER
		mRightShooterMotor.setControl(new Follower(mLeftShooterMotor.getDeviceID(), true));
		mLeftShooterMotor.setNeutralMode(NeutralModeValue.Brake);
		mRightShooterMotor.setNeutralMode(NeutralModeValue.Brake);

		SmartDashboard.putData("Shooter", mech);
	}

	/*-------------------------------- Generic Subsystem Functions --------------------------------*/

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
///* dc.2.10.25, commented out to compile so that we can merge GIT
				switch (mPeriodicIO.state) {
					case INTAKE:
						if (Laser.inRangeIntake()) {
							spin();
						} else {
							stop();
						}
						break;
					case SHOOT:
						if (Laser.inRangeShooter()) {
							spinFast();
						} else {
							intake();
						}
						break;
					default:
						System.err.println("coral shooter state corruption happened?");
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
		mLeftShooterMotor.set(mPeriodicIO.speed);

		if (Robot.isSimulation()) {
			updateSimPeriodic();
		}
	}

	@Override
	public void outputTelemetry() {

	}

	/*
	@Override
	public void reset() {
	}
	*/


	public void intake() {
		mPeriodicIO.state = ShooterState.INTAKE;
	}

	public void shoot() {
		mPeriodicIO.state = ShooterState.SHOOT;
	}

	public void spin() {
		mPeriodicIO.speed = Constants.CoralShooter.kShooterIntakeSpeed; //Constants.Shooter.kShooterSpeed;
	}

	public void spinFast() {
		mPeriodicIO.speed = Constants.CoralShooter.kShooterShootSpeed;
	}

	@Override
	public void stop() {
		mPeriodicIO.speed = 0.0;
	}

	// Update the simulation of the shooter.
  public void updateSimPeriodic() {
    // In this method, we update our simulation of what our shooter is doing
    TalonFXSimState mLeftShooterMotorSim = mLeftShooterMotor.getSimState();
    mLeftShooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    double leftShooterMotorVoltage = mLeftShooterMotorSim.getMotorVoltage();
    if (leftShooterMotorVoltage != 0.0) {
      // First, we set our "inputs" (voltages)
      shooterSim.setInput(leftShooterMotorVoltage);

      // Next, we update it for the standard loop time
      shooterSim.update(TimedRobot.kDefaultPeriod);

      // Finally, we set our simulated encoder's readings
      mLeftShooterMotorSim.setRawRotorPosition(shooterSim.getOutput(0));

      shooterViz.setLength(shooterViz.getLength() + shooterSim.getOutput(0));
    } else {
      // If the motor voltage is zero, reset the shooter length to original value
      shooterSim.setInput(0);
      shooterViz.setLength(-0.05);
    }
  }
}
