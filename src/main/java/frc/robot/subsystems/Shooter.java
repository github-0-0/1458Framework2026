package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Shooter mInstance;
  private PeriodicIO mPeriodicIO;

  public static Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
  }

  private TalonFX mLeftShooterMotor;
  private TalonFX mRightShooterMotor;


  private CANcoder mLeftShooterEncoder;
  private CANcoder mRightShooterEncoder;

  private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);

  private Shooter() {
    //super("Shooter");

    mPeriodicIO = new PeriodicIO();

    mLeftShooterMotor = new TalonFX(Constants.Shooter.kShooterLeftMotorId);//, MotorType.kBrushless);
    mRightShooterMotor = new TalonFX(Constants.Shooter.kShooterRightMotorId);//, MotorType.kBrushless);
    //mLeftShooterMotor.restoreFactoryDefaults();
    //mRightShooterMotor.restoreFactoryDefaults();
    
    
    mLeftShooterMotor.getConfigurator().apply(Constants.Shooter.ShooterConfiguration(),Constants.kLongCANTimeoutMs);
    mRightShooterMotor.getConfigurator().apply(Constants.Shooter.ShooterConfiguration(),Constants.kLongCANTimeoutMs);
    mLeftShooterEncoder = Cancoders.getInstance().getShooterLeft();
    mRightShooterEncoder = Cancoders.getInstance().getShooterRight();

    mLeftShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    mRightShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    //mLeftShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    //mRightShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    mLeftShooterMotor.setInverted(true);
    mRightShooterMotor.setInverted(false);
  }

  private static class PeriodicIO {
    double shooter_rpm = 0.0;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  /*
  @Override
  public void periodic() {
  }
  */

  @Override
  public void writePeriodicOutputs() {
    double limitedSpeed = mSpeedLimiter.calculate(mPeriodicIO.shooter_rpm);
    mLeftShooterMotor.set(limitedSpeed);//, ControlType.kVelocity);
    mRightShooterMotor.set(limitedSpeed);//, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    stopShooter();
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Speed (RPM):", mPeriodicIO.shooter_rpm);
    SmartDashboard.putNumber("Left speed:", mLeftShooterEncoder.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right speed:", mRightShooterEncoder.getVelocity().getValueAsDouble());
  }

  /*
  @Override
  public void reset() {
  }
  */

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void setSpeed(double rpm) {
    mPeriodicIO.shooter_rpm = rpm;
  }

  public void stopShooter() {
    mPeriodicIO.shooter_rpm = 0.0;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
