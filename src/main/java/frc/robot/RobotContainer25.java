package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Loops.Looper;
import frc.robot.autos.AutoModeBase;
import frc.robot.autos.AutoModeExecutor;
import frc.robot.autos.AutoModeSelector;
import frc.robot.autos.actions.SnapToTag;
import frc.robot.autos.modes.TeleopAutoMode;
//dc.2.11.25, keep Shooter for testing until CoralShooter is verified.
//dc.2.11.25, keep Shooter for testing until CoralShooter is verified.
import frc.robot.subsystems.*;
import frc.robot.subsystems.vision.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.util.Util;
import frc.robot.lib.trajectory.TrajectoryGenerator;
import frc.robot.Loops.CrashTracker;
import frc.robot.teleop.Controller;
/**
 * DC 10.28.2024
 * This class is where the bulk of the robot (for 2025 FRC season) should be
 * declared,
 * while very little robot logic should actually be handled in the {@link Robot}
 * periodic methods. Instead, the structure of the robot (including
 * subsystems, loopers, control button mappings etc) should be declared here.
 *
 */

public class RobotContainer25 {
    public static boolean is_red_alliance = false;  //TODO: code the update logic for this property

    private Controller m_Controller;

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
    private DummySubsystem m_ExampleSubsystem;
    private SwerveDrive m_SwerveDrive;
    private Elevator m_Elevator;
    //private Shooter m_Shooter; //replaced by CoralShooter
    private Cancoders m_Cancoders;
    //private AlgaeShooter m_AlgaeShooter;
    private CoralShooter m_CoralShooter;
    private Funnel m_Funnel;
    private Hang m_Hang;
    private LED m_Led;


    public AutoModeExecutor m_AutoModeExecutor;
    public static final AutoModeSelector m_AutoModeSelector = new AutoModeSelector();

    public AutoModeExecutor mTeleopActionExecutor;

    private VisionDeviceManager m_VisionDevices;

    // contructor
    public RobotContainer25() {
        try {
            // get instance of subsystems
            m_ExampleSubsystem = DummySubsystem.getInstance();
            m_Cancoders = Cancoders.getInstance();// Cancoders shall be initialized before SwerveDrive as Cancoders are
                                                  // used by Module constructor and initialization code
            m_SwerveDrive = SwerveDrive.getInstance();
            m_Elevator = Elevator.getInstance();
            //m_Shooter = Shooter.getInstance();//replaced by CoralShooter
        //    m_AlgaeShooter = AlgaeShooter.getInstance();
            m_CoralShooter = CoralShooter.getInstance();
            m_Led = LED.getInstance();

            //m_Hang = Hang.getInstance();
            //m_Funnel = Funnel.getInstance();

            //if (!Robot.isSimulation()){//turn off vision in simulation
                m_VisionDevices = VisionDeviceManager.getInstance();//}

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

            // add subsystems to its manager
            m_SubsystemManager.setSubsystems(
                    m_SwerveDrive,
                    m_Elevator,
                    m_ExampleSubsystem,
                    m_VisionDevices,
                    m_CoralShooter
//                    m_AlgaeShooter/* ,
                    // m_Hang,
                    // m_Funnel*/
            // Insert instances of additional subsystems here
            );
            // register subsystems to loopers
            m_SubsystemManager.registerEnabledLoops(m_EnabledLooper);
            m_SubsystemManager.registerDisabledLoops(m_DisabledLooper);

            RobotState.getInstance().resetKalman(); // TODO: complete RobotState classes

            // set robot to neutral brake
            if (m_SwerveDrive != null)
                m_SwerveDrive.setNeutralBrake(true);

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t); // TODO: CrashTracker needs to be ported. to log crash/exception
            throw t;
        }
    }

    // switch between two loopers
    public void switchOnLooper(Looper onLooper, Looper offLooper) {
        offLooper.stop();
        onLooper.start();
    }

    // init manual (teleop) mode
    public void initManualMode() {
        if (m_AutoModeExecutor != null) {m_AutoModeExecutor.stop();	}

   		try {

            // Create an empty TeleopAutoMode and bind it to controller
            mTeleopActionExecutor = new AutoModeExecutor();
            TeleopAutoMode teleopAutoMode = new TeleopAutoMode();
            mTeleopActionExecutor.setAutoMode(teleopAutoMode);
            m_Controller = new Controller(xboxController, teleopAutoMode, m_JoyStick);
            // turn on the looper
            //RobotState.getInstance().setIsInAuto(false);
            System.out.println("InitManualMode called");
            if (m_SwerveDrive != null)
                m_SwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(0.0, 0.0, 0.0));
            switchOnLooper(m_EnabledLooper, m_DisabledLooper);
            // start TeleopAutoMode, empty for now, to be activated by shortcut keys from controller.
            mTeleopActionExecutor.start();
		} catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

    }

    // init auto mode
    public void initAutoMode() {
        // get current auto mode from its menu selector
        // Reset all auto mode menu items.
		m_AutoModeSelector.reset();
		m_AutoModeSelector.updateModeCreator(true);
        Optional<AutoModeBase> autoMode = m_AutoModeSelector.getAutoMode();
        if (autoMode.isPresent()) {
            System.out.println("InitAuto mode =" + autoMode);
            m_AutoModeExecutor = new AutoModeExecutor();
            m_AutoModeExecutor.setAutoMode(autoMode.get());
        }
        //reset robot heading via gyro. robot has to orient at the right direction
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (!ally.isPresent()){return;}
		if (ally.get() == Alliance.Blue) {m_SwerveDrive.zeroGyro(180);
        }else{m_SwerveDrive.zeroGyro(0);}

        try {
            // RobotState.getInstance().setIsInAuto(false);
            switchOnLooper(m_EnabledLooper, m_DisabledLooper);
            m_AutoModeExecutor.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

    }

    // init diabled  mode, this is the staging place to get ready for other modes, such as auto
    public void initDisabledMode() {
        //close current auto mode
        if (m_AutoModeExecutor != null) {
            m_AutoModeExecutor.stop();
        }
        // turn on loopers
        try {
            switchOnLooper(m_DisabledLooper, m_EnabledLooper);

            xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public void disabledPeriodicMode() {
    m_Led.rainbowPulse();
        //reset robot heading via gyro. robot has to orient at the right direction
        if (foundStation) return;

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

    // init manual (teleop) mode
    public void initTestMode() {
        try {
            // RobotState.getInstance().setIsInAuto(false);
            if (m_AutoModeExecutor != null) {
                m_AutoModeExecutor.stop();
            }

            // testChassisSpeedConvert();
            // CrashTracker.logTest("Testing crashtracker - if you see this it works");
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public void testModePeriodic() {
        Laser.testLaser();
    }

    // manual mode periodic callback
    public void manualModePeriodic() {
        try {
            // mControlBoard.update();

            /* Drive */

            // if (xboxController.getStartButton()) {
            //     System.out.println("keyY is pressed, zero the wheels!");
            //     if (m_SwerveDrive != null)
            //         m_SwerveDrive.zeroGyro(0);
			// }
                //dc.11.9.24, to scale up joystick input to max-speed
                m_Controller.processKeyCommand();

                // for(int i = 0; i < 4;  i++) {
                //     SmartDashboard.putBoolean("Mag Sensor " + i, DigitalSensor.getSensor(i));
                // }


            // mDriverControls.oneControllerMode();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

    }

    public void testChassisSpeedConvert() {
        double tV = 2;
        double sV = 0.0;// 1;
        double rV = 0.0;
        ChassisSpeeds rs = ChassisSpeeds.fromFieldRelativeSpeeds(tV, sV, rV,
                Util.robotToFieldRelative(m_SwerveDrive.getHeading(), is_red_alliance));
        System.out.println("DC: testChassisSpeedConvert field speed: tVal=" + tV + ", sVal=" + sV + ", rVal=" + rV);
        System.out.println("DC: testChassisSpeedConvert robot speed: tVal=" + rs.vxMetersPerSecond + ", sVal="
                + rs.vyMetersPerSecond + ", rVal=" + rs.omegaRadiansPerSecond);
        System.out.println("DC: testChassisSpeedConvert swerveHeading: heading=" + m_SwerveDrive.getHeading()
                + ", field=" + sV + ", rVal=" + rV);
    }
/*
    // dummy methods for now.
    public Command getAutonomousCommand() {
        return null;
    }

    public void updateLimeLightData() {
    }
*/
    //init robot for simulation mode
    private void initSimulation(){
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
