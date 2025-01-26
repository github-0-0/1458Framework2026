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

public class Shooter extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Shooter mInstance;
  
  private PeriodicIO mPeriodicIO = new PeriodicIO();

  private double minSpeed = 0;
  private double speed = 0;

  public static Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
  }

  private class PeriodicIO {
    double speed = 0.0;
    boolean on = false;
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

  @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
      enabledLooper.register(new Loop() {
      @Override
      public void onStart(double timestamp) {

      }

      @Override
      public void onLoop(double timestamp) {
        if(mPeriodicIO.on) {
          if(checkSensor()) {
            spin();
          }
          else 
          {
            stop();
          }
        }
        else
        {
          stop();
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
    mRightShooterMotor.set(mPeriodicIO.speed);
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
    mPeriodicIO.speed = 0.05;
  }
  
  @Override
  public void stop() {
    mPeriodicIO.speed = 0.0;
  }

  public boolean checkSensor() {
    if(Laser.inRangeIntake()) {
      return true;
    }
    else {
      return false;
    }
  }
  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
