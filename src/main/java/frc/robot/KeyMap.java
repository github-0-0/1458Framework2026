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
import frc.robot.actions.*;
import frc.robot.lib.util.Util;

/**
 * A class that contains the controls for the robot.
 */
public class KeyMap {
    private XboxController mXboxController1 = null;
    private ActionExecutor mTeleopActionExecutor = null;
    private Joystick mJoyStick = null;
    private Drive mSwerveDrive = null;
    private VisionDeviceManager mVisionDevices = null;
    private LED mLED = null;

    private final EventLoop mLoop = new EventLoop();
    
    private boolean isFieldRelative = true;

    private final int TRANSLATION_AXIS = XboxController.Axis.kLeftY.value;
    private final int STRAFE_AXIS = XboxController.Axis.kLeftX.value;
    private final int ROTATION_AXIS = XboxController.Axis.kRightX.value;

    public KeyMap(
        XboxController xboxController1, 
        ActionExecutor teleopAutoMode,
        Joystick joystick
    ) {
        mXboxController1 = xboxController1;
        mTeleopActionExecutor = teleopAutoMode;
        mJoyStick = joystick;
        mSwerveDrive = Drive.getInstance();
        mVisionDevices = VisionDeviceManager.getInstance();
        mLED = LED.getInstance();
        
        //XYAB
        mXboxController1.a(mLoop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));
        mXboxController1.b(mLoop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));
        mXboxController1.x(mLoop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));
        mXboxController1.y(mLoop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));
        //Bumper
        mXboxController1.leftBumper(mLoop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new SnapToTag("CENTER", "ANY")));
        mXboxController1.rightBumper(mLoop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));
        //Triggers
        mXboxController1.rightTrigger(0.5, mLoop).ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));
        mXboxController1.leftTrigger(0.5, mLoop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));
        //Misc
        mXboxController1.button(7, mLoop).rising().ifHigh(
                () -> mTeleopActionExecutor.runAction(new EmptyAction()));

        mXboxController1.start(mLoop).ifHigh(
                () -> isFieldRelative = !isFieldRelative
        );
    }

    public void processKeyCommand() {
        if (mTeleopActionExecutor == null)
            return;

        mLoop.poll();

        double translationVal = -MathUtil.applyDeadband(mJoyStick.getRawAxis(TRANSLATION_AXIS), Constants.STICK_DEADBAND)
                * Constants.Swerve.MAX_SPEED;
        double strafeVal = -MathUtil.applyDeadband(mJoyStick.getRawAxis(STRAFE_AXIS), Constants.STICK_DEADBAND)
                * Constants.Swerve.MAX_SPEED;
        double rotationVal = -MathUtil.applyDeadband(mJoyStick.getRawAxis(ROTATION_AXIS), Constants.STICK_DEADBAND)
                * Constants.Swerve.MAX_ANGULAR_VELOCITY;

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            translationVal = -translationVal;
            strafeVal = -strafeVal;
        }

        if (mXboxController1.getPOV() == 90) {
            mSwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                    0, -0.4, 0));
        } else if (mXboxController1.getPOV() == 0) {
            mSwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                    0.4, 0, 0));
        } else if (mXboxController1.getPOV() == 270) {
            mSwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                    0, 0.4, 0));
        } else if (mXboxController1.getPOV() == 180) {
            mSwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                    -0.4, 0, 0));
        } else {
            if (isFieldRelative) {
                mSwerveDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationVal, strafeVal, rotationVal,
                        Util.robotToFieldRelative(mSwerveDrive.getHeading(), DriverStation.getAlliance().get() == Alliance.Red)));
            } else {
                mSwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                        translationVal, strafeVal, rotationVal));
            }
        }
    }
}
