package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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




  private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);

  private Shooter() {
    //super("Shooter");


    mLeftShooterMotor = new TalonFX(Constants.Shooter.kShooterLeftMotorId);
    mRightShooterMotor = new TalonFX(Constants.Shooter.kShooterRightMotorId); //LEADER
    mLeftShooterMotor.setControl(new Follower(mRightShooterMotor.getDeviceID(), true));
    mRightShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    mRightShooterMotor.setNeutralMode(NeutralModeValue.Brake);
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
  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
