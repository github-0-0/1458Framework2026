package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.subsystems.*;
import frc.robot.subsystems.vision.VisionDeviceManager;
import frc.robot.autos.actions.*;
import frc.robot.lib.util.Util;

/**
 * A class that contains the controls for the robot.
 */
public class KeyMap {
    private XboxController mXboxController1 = null;
    private TeleopActionExecutor mTeleopActionExecutor = null;
    private Joystick m_JoyStick = null;
    private Drive m_SwerveDrive = null;
    private VisionDeviceManager m_VisionDevices = null;
    private LED m_LED = null;

    private final EventLoop m_loop = new EventLoop();
    
    private boolean isFieldRelative = true;

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    public KeyMap(
        XboxController xboxController1, 
        TeleopActionExecutor teleopAutoMode,
        Joystick joystick
    ) {
        mXboxController1 = xboxController1;
        mTeleopActionExecutor = teleopAutoMode;
        m_JoyStick = joystick;
        m_SwerveDrive = Drive.getInstance();
        m_VisionDevices = VisionDeviceManager.getInstance();
        m_LED = LED.getInstance();
        
        //XYAB
        mXboxController1.a(m_loop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));
        mXboxController1.b(m_loop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));
        mXboxController1.x(m_loop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));
        mXboxController1.y(m_loop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));
        //Bumper
        mXboxController1.leftBumper(m_loop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new SnapToTag("CENTER", "ANY")));
        mXboxController1.rightBumper(m_loop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));
        //Triggers
        mXboxController1.rightTrigger(0.5, m_loop).ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));
        mXboxController1.leftTrigger(0.5, m_loop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));
        //Misc
        mXboxController1.button(7, m_loop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));

        mXboxController1.start(m_loop).ifHigh(
                () -> isFieldRelative = !isFieldRelative
        );
    }

    public void processKeyCommand() {
        if (mTeleopActionExecutor == null)
            return;

        m_loop.poll();

        double translationVal = -MathUtil.applyDeadband(m_JoyStick.getRawAxis(translationAxis), Constants.STICK_DEADBAND)
                * Constants.Swerve.MAX_SPEED;
        double strafeVal = -MathUtil.applyDeadband(m_JoyStick.getRawAxis(strafeAxis), Constants.STICK_DEADBAND)
                * Constants.Swerve.MAX_SPEED;
        double rotationVal = -MathUtil.applyDeadband(m_JoyStick.getRawAxis(rotationAxis), Constants.STICK_DEADBAND)
                * Constants.Swerve.MAX_ANGULAR_VELOCITY;

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            translationVal = -translationVal;
            strafeVal = -strafeVal;
        }

        if (mXboxController1.getPOV() == 90) {
            m_SwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                    0, -0.4, 0));
        } else if (mXboxController1.getPOV() == 0) {
            m_SwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                    0.4, 0, 0));
        } else if (mXboxController1.getPOV() == 270) {
            m_SwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                    0, 0.4, 0));
        } else if (mXboxController1.getPOV() == 180) {
            m_SwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                    -0.4, 0, 0));
        } else {
            if (isFieldRelative) {
                m_SwerveDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationVal, strafeVal, rotationVal,
                        Util.robotToFieldRelative(m_SwerveDrive.getHeading(), DriverStation.getAlliance().get() == Alliance.Red)));
            } else {
                m_SwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                        translationVal, strafeVal, rotationVal));
            }
        }
    }
}
