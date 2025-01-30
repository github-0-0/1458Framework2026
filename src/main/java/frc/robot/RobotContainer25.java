package frc.robot;


import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Loops.Looper;
import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeExecutor;
import frc.robot.autos.AutoModeSelector;
import frc.robot.subsystems.Cancoders;
import frc.robot.subsystems.DummySubsystem;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.vision.VisionDeviceManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.util.Util;
import frc.robot.lib.trajectory.TrajectoryGenerator;
import frc.robot.Loops.CrashTracker;
/**
 * DC 10.28.2024
 * This class is where the bulk of the robot (for 2025 FRC season) should be declared,
 * while very little robot logic should actually be handled in the {@link Robot}
 * periodic methods. Instead, the structure of the robot (including
 * subsystems, loopers, control button mappings etc) should be declared here.
 *
 */

public class RobotContainer25 {
    public static boolean is_red_alliance = false;  //TODO: code the update logic for this property

    /* Controllers */
    private final Joystick m_JoyStick = new Joystick(0);
    /* button key-value */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    /* JoyStick Buttons */
    private final JoystickButton m_btnZeroGyro = new JoystickButton(m_JoyStick, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(m_JoyStick, XboxController.Button.kLeftBumper.value);//TODO: how to drive if not robot-centric?

    /* loop framework objects*/
    private final Looper m_EnabledLooper = new Looper();
    private final Looper m_DisabledLooper = new Looper();
    public final SubsystemManager m_SubsystemManager = SubsystemManager.getInstance();
    /* Subsystems instance */
    private DummySubsystem m_ExampleSubsystem;
    public SwerveDrive m_SwerveDrive=null;
    private Cancoders m_Cancoders;
    
    public AutoModeExecutor m_AutoModeExecutor;
    public static final AutoModeSelector m_AutoModeSelector = new AutoModeSelector();

    private VisionDeviceManager m_VisionDevices = VisionDeviceManager.getInstance();
	
    //contructor
    public RobotContainer25 (){
        try{
            //get instance of subsystems
            m_ExampleSubsystem = DummySubsystem.getInstance();
            if (!Constants.isBareboneRobot) {//dc.1.29.2025 disable swervedrive initialization on the barebone code
                m_Cancoders = Cancoders.getInstance();//Cancoders shall be initialized before SwerveDrive as Cancoders are used by Module constructor and initialization code
                m_SwerveDrive = SwerveDrive.getInstance();
            }    

            // init cancoders
            if (Robot.isReal() && !Constants.isBareboneRobot) {
                m_Cancoders = Cancoders.getInstance();
                double startInitTs = Timer.getFPGATimestamp();
                System.out.println("* Starting to init Cancoders at ts " + startInitTs);
                while (Timer.getFPGATimestamp() - startInitTs < Constants.SwerveConstants.kCancoderBootAllowanceSeconds
                        && !m_Cancoders.allHaveBeenInitialized()) {
                    Timer.delay(0.1);
                }
                System.out.println(
                        "* Cancoders all initialized: Took " + (Timer.getFPGATimestamp() - startInitTs) + " seconds");
            }

            // reset swerve modules
            if (m_SwerveDrive != null)             
                m_SwerveDrive.resetModulesToAbsolute();

            //add subsystems to its manager
            m_SubsystemManager.setSubsystems(
//                m_SwerveDrive,//dc.1.29.2025 disable swervedrive initialization on the barebone code
                m_ExampleSubsystem,
                m_VisionDevices
                //Insert instances of additional subsystems here
            );
            //register subsystems to loopers
            m_SubsystemManager.registerEnabledLoops(m_EnabledLooper);
            m_SubsystemManager.registerDisabledLoops(m_DisabledLooper);

            //load all predefined trajectories  
            TrajectoryGenerator.getInstance().generateTrajectories();
			/*
            RobotState.getInstance().resetKalman(); //TODO: complete RobotState classes
            */
            //set robot to neutral brake
            if (m_SwerveDrive != null)             
                m_SwerveDrive.setNeutralBrake(true);

            //binds single-button events
//            bindSingleButtonCmds ();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);    //TODO: CrashTracker needs to be ported. to log crash/exception
			throw t;
		}
    }

    /**
     * Use this method to define your button->command mappings for single-button events. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void bindSingleButtonCmds() {
        System.out.println("-->binding single button commands");
//        m_btnZeroGyro.onTrue(new InstantCommand(() -> m_SwerveDrive.resetModulesToAbsolute())); //TODO: zeroGyro() vs zeroHeading()? check with victor
        // additional command bindings for single-event buttons
    }


    // switch between two loopers
    public void switchOnLooper (Looper onLooper, Looper offLooper){
        offLooper.stop();
        onLooper.start();
    }

    // init manual (teleop) mode
    public void initManualMode (){
        if (m_AutoModeExecutor != null) {
			m_AutoModeExecutor.stop();
		}
   		try {
//          RobotState.getInstance().setIsInAuto(false);
            System.out.println("InitManualMode called");
            if (m_SwerveDrive != null)             
     			m_SwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(0.0, 0.0, 0.0));
            switchOnLooper(m_EnabledLooper, m_DisabledLooper);
		} catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
			throw t;
		}

    }
    // init manual (teleop) mode
    public void initAutoMode (){
        try {
//          RobotState.getInstance().setIsInAuto(false);
            switchOnLooper(m_EnabledLooper, m_DisabledLooper);
            
            m_AutoModeExecutor.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

    }

    // init manual (teleop) mode
    public void initDisabledMode (){
        if (m_AutoModeExecutor != null) {
			m_AutoModeExecutor.stop();
		}
		m_AutoModeSelector.reset();
		m_AutoModeSelector.updateModeCreator(false);
        Optional<AutoModeBase> autoMode = m_AutoModeSelector.getAutoMode();

		m_AutoModeExecutor = new AutoModeExecutor();
        if (autoMode.isPresent() && (autoMode.get() != m_AutoModeExecutor.getAutoMode())) {
            m_AutoModeExecutor.setAutoMode(autoMode.get());
        }
        try {
            switchOnLooper(m_DisabledLooper, m_EnabledLooper);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
    }

    // init manual (teleop) mode
    public void initTestMode (){
        try {
//          RobotState.getInstance().setIsInAuto(false);
            if (m_AutoModeExecutor != null) {
			    m_AutoModeExecutor.stop();
		    }
            CrashTracker.logTest("Testing crashtracker - if you see this it works");
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
        /*try {
            System.out.println("InitTestMode called");
//            m_SwerveDrive.straightenAllWheels();
//            try{Thread.sleep(3000);}catch(Exception e){}
//            m_SwerveDrive.testSwerve();
            if (m_AutoModeExecutor != null) {
			    m_AutoModeExecutor.stop();
		    }
            m_DisabledLooper.stop();
            m_EnabledLooper.stop();
		} catch (Throwable t) {
//			CrashTracker.logThrowableCrash(t);
			throw t;
		}*/
    }

    // manual mode periodic callback
    public void manualModePeriodic (){
		try {
//			mControlBoard.update();

			/* Drive */
			if (m_JoyStick.getRawButton(XboxController.Button.kStart.value)) {
				System.out.println("keyY is pressed, zero the wheels!");
                if (m_SwerveDrive != null)             
                    m_SwerveDrive.zeroGyro(0);
			}

            //dc.11.9.24, to scale up joystick input to max-speed
            double translationVal = -MathUtil.applyDeadband(m_JoyStick.getRawAxis(translationAxis), Constants.stickDeadband)*Constants.SwerveConstants.maxSpeed;
            double strafeVal = - MathUtil.applyDeadband(m_JoyStick.getRawAxis(strafeAxis), Constants.stickDeadband)*Constants.SwerveConstants.maxSpeed;
            double rotationVal = MathUtil.applyDeadband(m_JoyStick.getRawAxis(rotationAxis), Constants.stickDeadband)* Constants.Swerve.maxAngularVelocity;
//                System.out.println("DC: manualModePeriodc() translationVal=" + translationVal + ", StrafeVal=" + strafeVal + ", rotationVal=" + rotationVal);
            if (m_SwerveDrive != null)             
                m_SwerveDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
                    translationVal, strafeVal, rotationVal,
                    Util.robotToFieldRelative(m_SwerveDrive.getHeading(), is_red_alliance)));


//			mDriverControls.oneControllerMode();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

    }

    //dummy methods for now.
    public Command getAutonomousCommand() {
        return null;
    }
    public void updateLimeLightData() {
    }
}
