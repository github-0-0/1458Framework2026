package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
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

public class Shooter extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Shooter mInstance;
  private double minSpeed = 0;
  private double speed = 0;
  private boolean sensorTripped = false;

  public static Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
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
            "Shaft", -0.05, 0, 10.0, new Color8Bit(Color.kBlue)
        )
      );

  private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);

  private Shooter() {
    //super("Shooter");


    mLeftShooterMotor = new TalonFX(Constants.Shooter.kShooterLeftMotorId);
    mRightShooterMotor = new TalonFX(Constants.Shooter.kShooterRightMotorId); //LEADER
    mLeftShooterMotor.setControl(new Follower(mRightShooterMotor.getDeviceID(), true));
    mRightShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    mRightShooterMotor.setNeutralMode(NeutralModeValue.Brake);

    SmartDashboard.putData("Shooter", mech);
  }


  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  /*
  @Override
  public void periodic() {
  }
  */

  @Override
  public void writePeriodicOutputs() {

  }

  @Override
  public void stop() {
    mRightShooterMotor.set(0);
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

  public void spin() {
    mRightShooterMotor.set(-0.05);
  }

  public void spinFast() {
    mRightShooterMotor.set(-0.15);
  }

  public void reverse() {
    mRightShooterMotor.set(0.05);
  }

  public void checkSensor() {
    if(Laser.inRangeIntake()) {
      spin();
      sensorTripped = true;
    }
    else if(sensorTripped == true) {
      stop();
      sensorTripped = false;
    }
  }

  // Update the simulation of the shooter.
  public void updateSimPeriodic() {
    // In this method, we update our simulation of what our shooter is doing
    TalonFXSimState mRightShooterMotorSim = mRightShooterMotor.getSimState();
    mRightShooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    double rightShooterMotorVoltage = mRightShooterMotorSim.getMotorVoltage();
    if (rightShooterMotorVoltage != 0.0) {
      // First, we set our "inputs" (voltages)
      shooterSim.setInput(rightShooterMotorVoltage);

      // Next, we update it for the standard loop time
      shooterSim.update(TimedRobot.kDefaultPeriod);

      // Finally, we set our simulated encoder's readings
      mRightShooterMotorSim.setRawRotorPosition(shooterSim.getOutput(0));

      shooterViz.setLength(shooterViz.getLength() + shooterSim.getOutput(0));
    } else {
      // If the motor voltage is zero, reset the shooter length to original value
      shooterSim.setInput(0);
      shooterViz.setLength(-0.05);
    }
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}