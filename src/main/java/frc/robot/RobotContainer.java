package frc.robot;

import java.util.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.autos.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.vision.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.loops.CrashTracker;
import frc.robot.lib.loops.Looper;
import frc.robot.lib.util.Util;
import frc.robot.lib.util.interpolation.InterpolatingPose2d;
/**
 * This class is where the bulk of the robot should be
 * declared,
 * while very little robot logic should actually be handled in the {@link Robot}
 * periodic methods. Instead, the structure of the robot (including
 * subsystems, loopers, control button mappings etc) should be declared here.
 */
public class RobotContainer {
    public static boolean mIsRedAlliance = false;  //TODO: code the update logic for this property

    private KeyMap mController;
    
    private Boolean mFoundStation = false;

    /* Controllers */
    private final Joystick mJoyStick = new Joystick(0);

    private final XboxController mXboxController = new XboxController(0);
    
    /* loop framework objects */
    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();
    public final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    /* Subsystems instance */
    private ExampleSubsystem mExampleSubsystem;
    private Drive mSwerveDrive;
    private LED mLed;
    private Cancoders mCancoders;

    public AutoModeExecutor mAutoModeExecutor;
    public static final AutoModeSelector mAutoModeSelector = new AutoModeSelector();

    public AutoModeExecutor mTeleopAutoExecutor;
	
    private VisionDeviceManager mVisionDevices;

    public RobotContainer() {
        try {
            // initialize subsystems
            mExampleSubsystem = ExampleSubsystem.getInstance();
            mSwerveDrive = Drive.getInstance();
            mLed = LED.getInstance();
            mVisionDevices = VisionDeviceManager.getInstance();

            // initialize cancoders
            if (Robot.isReal()) {
                mCancoders = Cancoders.getInstance();
                double startInitTs = Timer.getFPGATimestamp();
                System.out.println("* Starting to init Cancoders at ts " + startInitTs);
                while (Timer.getFPGATimestamp() - startInitTs < Constants.Swerve.CANCODER_BOOT_ALLOWANCE_SECONDS
                        && !mCancoders.allHaveBeenInitialized()) {
                    Timer.delay(0.1);
                }
                System.out.println(
                        "* Cancoders all initialized: Took " + (Timer.getFPGATimestamp() - startInitTs) + " seconds");
            }

            // reset swerve modules
            if (mSwerveDrive != null)
                mSwerveDrive.resetModulesToAbsolute();

            // add subsystems to SubsystemManager
            mSubsystemManager.setSubsystems(
                    mSwerveDrive,
                    mExampleSubsystem,
                    mVisionDevices
                    // TODO: Insert instances of additional subsystems here
            );

            // register subsystems to loopers
            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);


		    RobotState.reset(0.0, new InterpolatingPose2d());
            RobotState.resetKalman();

            // set swerves to neutral brake
            if (mSwerveDrive != null)
                mSwerveDrive.setNeutralBrake(true);

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * Change to using onLooper
     * @param onLooper the onLooper object
     * @param offLooper the offLooper object
     */
    public void switchOnLooper(Looper onLooper, Looper offLooper) {
        offLooper.stop();
        onLooper.start();
    }

    /**
     * Runs at the start of teleop mode
     */
    public void initTeleopMode() {
        if (mAutoModeExecutor != null) 
            mAutoModeExecutor.stop();  
        
   		try {
            // Create an empty TeleopAutomation and bind it to controller
            TeleopActionExecutor mTeleopActionExecutor = new TeleopActionExecutor();
            mController = new KeyMap(mXboxController, mTeleopActionExecutor, mJoyStick);
            System.out.println("Initializing Teleop Mode");
            if (mSwerveDrive != null)
                mSwerveDrive.feedTeleopSetpoint(new ChassisSpeeds());
            switchOnLooper(mEnabledLooper, mDisabledLooper);
            mTeleopAutoExecutor.start();
		} catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * Runs every cycle in teleop mode
     */
    public void teleopModePeriodic() {
        try {
            mController.processKeyCommand();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * Runs at the beginning of autonomous mode
     */
    public void initAutoMode() {
		mAutoModeSelector.reset();
		mAutoModeSelector.updateModeCreator(true);
        Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
        if (autoMode.isPresent()) {
            System.out.println("Initializing Auto Mode: " + autoMode.get().getClass().getName());
            mAutoModeExecutor = new AutoModeExecutor();
            mAutoModeExecutor.setAutoMode(autoMode.get());
        }

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (!ally.isPresent())
            return;
		if (ally.get() == Alliance.Blue) {
            mSwerveDrive.zeroGyro(180);
        } else {
            mSwerveDrive.zeroGyro(0);
        }
        
        try {
            switchOnLooper(mEnabledLooper, mDisabledLooper);
            mAutoModeExecutor.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
    
    /**
     * Runs at the beginning of disabled mode
     */
    public void initDisabledMode() {
        if (mAutoModeExecutor != null) {
            mAutoModeExecutor.stop();
        }

        try {
            switchOnLooper(mDisabledLooper, mEnabledLooper);
            mXboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * Runs every cycle in disabled mode
     */
    public void disabledPeriodicMode() {
        if (mFoundStation) 
            return;

        Optional<Alliance> ally = DriverStation.getAlliance();

        if (!ally.isPresent()){
            return;
        } if (ally.get() == Alliance.Blue) {
            mSwerveDrive.zeroGyro(180);
        } else {
            mSwerveDrive.zeroGyro(0);
        }

        if (RobotState.mLatestVisionUpdate.isPresent()) {
            mSwerveDrive.resetOdometry(new Pose2d(
                    RobotState.mLatestVisionUpdate.get().getFieldToVehicle(),
                    Rotation2d.kPi
            ));
        }

        try {
            mLed.writePeriodicOutputs();
        } catch (Exception e) {}

        mFoundStation = true;
    }

    /**
     * Runs at the beginning of test mode
     */
    public void initTestMode() {
        try {
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * Runs every cycle in test mode
     */
    public void testModePeriodic() {}

    /** 
     * Runs at the beginning of simulation mode
    */
    public void initSimulation() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (!ally.isPresent()){
            System.out.println("Alliance is NOT present! Abort!");
            return;
        }
		if (ally.get() == Alliance.Blue) {
            mSwerveDrive.zeroGyro(180);
            mSwerveDrive.resetOdometry(new Pose2d(new Translation2d(8.0,7.0), Rotation2d.fromDegrees(0)));
        } else {
            mSwerveDrive.zeroGyro(0);
            mSwerveDrive.resetOdometry(new Pose2d(new Translation2d(9.5,1.26), Rotation2d.fromDegrees(0)));
        }
    }

    /**
     * Runs every cycle in simulation mode
     */
    public void simulationPeriodic() {}
}
