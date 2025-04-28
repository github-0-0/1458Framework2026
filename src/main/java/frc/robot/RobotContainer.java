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
import frc.robot.autos.modes.TeleopAutoMode;
import frc.robot.subsystems.*;
import frc.robot.subsystems.vision.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.loops.CrashTracker;
import frc.robot.lib.loops.Looper;
import frc.robot.lib.util.Util;
/**
 * This class is where the bulk of the robot should be
 * declared,
 * while very little robot logic should actually be handled in the {@link Robot}
 * periodic methods. Instead, the structure of the robot (including
 * subsystems, loopers, control button mappings etc) should be declared here.
 */
public class RobotContainer {
    public static boolean is_red_alliance = false;  //TODO: code the update logic for this property

    private KeyMap m_Controller;
    
    private Boolean foundStation = false;

    /* Controllers */
    private final Joystick m_JoyStick = new Joystick(0);

    private final XboxController xboxController = new XboxController(0);
    //private final XboxController xboxController2 = new XboxController(1);
    /* button key-value */
    /* JoyStick Buttons */
    
    /* loop framework objects */
    private final Looper m_EnabledLooper = new Looper();
    private final Looper m_DisabledLooper = new Looper();
    public final SubsystemManager m_SubsystemManager = SubsystemManager.getInstance();
    /* Subsystems instance */
    private ExampleSubsystem m_ExampleSubsystem;
    private Drive m_SwerveDrive;
    private LED m_Led;
    private Cancoders m_Cancoders;

    public AutoModeExecutor m_AutoModeExecutor;
    public static final AutoModeSelector m_AutoModeSelector = new AutoModeSelector();

    public AutoModeExecutor mTeleopActionExecutor;
	
    private VisionDeviceManager m_VisionDevices;

    // contructor
    public RobotContainer() {
        try {
            // initialize subsystems
            m_ExampleSubsystem = ExampleSubsystem.getInstance();
            m_SwerveDrive = Drive.getInstance();
            m_Led = LED.getInstance();
            m_VisionDevices = VisionDeviceManager.getInstance();

            // initialize cancoders
            if (Robot.isReal()) {
                m_Cancoders = Cancoders.getInstance();
                double startInitTs = Timer.getFPGATimestamp();
                System.out.println("* Starting to init Cancoders at ts " + startInitTs);
                while (Timer.getFPGATimestamp() - startInitTs < Constants.Swerve.kCancoderBootAllowanceSeconds
                        && !m_Cancoders.allHaveBeenInitialized()) {
                    Timer.delay(0.1);
                }
                System.out.println(
                        "* Cancoders all initialized: Took " + (Timer.getFPGATimestamp() - startInitTs) + " seconds");
            }

            // reset swerve modules
            if (m_SwerveDrive != null)
                m_SwerveDrive.resetModulesToAbsolute();

            // add subsystems to SubsystemManager
            m_SubsystemManager.setSubsystems(
                    m_SwerveDrive,
                    m_ExampleSubsystem,
                    m_VisionDevices
                    // TODO: Insert instances of additional subsystems here
            );

            // register subsystems to loopers
            m_SubsystemManager.registerEnabledLoops(m_EnabledLooper);
            m_SubsystemManager.registerDisabledLoops(m_DisabledLooper);

            RobotState.getInstance().resetKalman();

            // set swerves to neutral brake
            if (m_SwerveDrive != null)
                m_SwerveDrive.setNeutralBrake(true);

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
        if (m_AutoModeExecutor != null) 
            m_AutoModeExecutor.stop();  
        
   		try {
            // Create an empty TeleopAutoMode and bind it to controller
            mTeleopActionExecutor = new AutoModeExecutor();
            TeleopAutoMode teleopAutoMode = new TeleopAutoMode();
            mTeleopActionExecutor.setAutoMode(teleopAutoMode);
            m_Controller = new KeyMap(xboxController, teleopAutoMode, m_JoyStick);
            System.out.println("Initializing Manual Mode");
            if (m_SwerveDrive != null)
                m_SwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(0.0, 0.0, 0.0));
            switchOnLooper(m_EnabledLooper, m_DisabledLooper);
            mTeleopActionExecutor.start();
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
            m_Controller.processKeyCommand();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * Runs at the beginning of autonomous mode
     */
    public void initAutoMode() {
		m_AutoModeSelector.reset();
		m_AutoModeSelector.updateModeCreator(true);
        Optional<AutoModeBase> autoMode = m_AutoModeSelector.getAutoMode();
        if (autoMode.isPresent()) {
            System.out.println("Initializing Auto Mode: " + autoMode.get().getClass().getName());
            m_AutoModeExecutor = new AutoModeExecutor();
            m_AutoModeExecutor.setAutoMode(autoMode.get());
        }

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (!ally.isPresent())
            return;
		if (ally.get() == Alliance.Blue) {
            m_SwerveDrive.zeroGyro(180);
        } else {
            m_SwerveDrive.zeroGyro(0);
        }
        
        try {
            switchOnLooper(m_EnabledLooper, m_DisabledLooper);
            m_AutoModeExecutor.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
    
    /**
     * Runs at the beginning of disabled mode
     */
    public void initDisabledMode() {
        if (m_AutoModeExecutor != null) {
            m_AutoModeExecutor.stop();
        }
        try {
            switchOnLooper(m_DisabledLooper, m_EnabledLooper);
            xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * Runs every cycle in disabled mode
     */
    public void disabledPeriodicMode() {
        m_Led.rainbowPulse();
        if (foundStation) 
            return;

        Optional<Alliance> ally = DriverStation.getAlliance();

        if (!ally.isPresent()){
            return;
        } if (ally.get() == Alliance.Blue) {
            m_SwerveDrive.zeroGyro(180);
        } else {
            m_SwerveDrive.zeroGyro(0);
        }

        if (RobotState.getInstance().mLatestVisionUpdate.isPresent()) {
            m_SwerveDrive.resetOdometry(new Pose2d(RobotState.getInstance().mLatestVisionUpdate.get().getFieldToVehicle(), Rotation2d.fromDegrees(180)));
        }

        foundStation = true;
    }

    
    public void initTestMode() {
        try {
            if (m_AutoModeExecutor != null) {
                m_AutoModeExecutor.stop();
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public void testModePeriodic() {
        
    }



    /** 
     * init robot for simulation mode
    */
    private void initSimulation() {
        //put robot on the start line  to simulate the actual game
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (!ally.isPresent()){
            System.out.println("Alliance is NOT present! Abort!");
            return;
        }
		if (ally.get() == Alliance.Blue) {
            m_SwerveDrive.zeroGyro(180);
            m_SwerveDrive.resetOdometry(new Pose2d(new Translation2d(8.0,7.0), Rotation2d.fromDegrees(0)));
        }else{
            m_SwerveDrive.zeroGyro(0);
            m_SwerveDrive.resetOdometry(new Pose2d(new Translation2d(9.5,1.26), Rotation2d.fromDegrees(0)));
        }
    }
}
