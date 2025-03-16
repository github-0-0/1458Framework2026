package frc.robot.teleop;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.subsystems.*;
import frc.robot.subsystems.vision.VisionDeviceManager;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.RobotState;
import frc.robot.autos.actions.*;
import frc.robot.autos.modes.TeleopAutoMode;
import frc.robot.lib.util.Util;

/**
 * A class that contains the controls for the robot.
 */
public class Controller {
    private XboxController mXboxController1 = null;
    private TeleopAutoMode mTeleopAutoMode = null;
    private Joystick m_JoyStick = null;
    private SwerveDrive m_SwerveDrive = null;
    private VisionDeviceManager m_VisionDevices = null;
    private LED m_LED = null;

    private final EventLoop m_loop = new EventLoop();
    
    private boolean isFieldRelative = true;

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    public Controller(
        XboxController xboxController1, 
        TeleopAutoMode teleopAutoMode,
        Joystick joystick
    ) {
        mXboxController1 = xboxController1;
        mTeleopAutoMode = teleopAutoMode;
        m_JoyStick = joystick;
        m_SwerveDrive = SwerveDrive.getInstance();
        m_VisionDevices = VisionDeviceManager.getInstance();
        m_LED = LED.getInstance();
                                                                                        //COMMENTS ARE ALGAE MODE
        mXboxController1.a(m_loop).rising().ifHigh(                                     //On shoot button, move pivot to proc. and shoot
                () -> mTeleopAutoMode.runAction(new ElevatorAction("Ground")));    //On intake button, move pivot to ground and intake
        mXboxController1.b(m_loop).rising().ifHigh(
                () -> mTeleopAutoMode.runAction(new ElevatorAction("L2")));         //Auto move pivot to ground, on intake button intake, shoot shoots
        mXboxController1.x(m_loop).rising().ifHigh(
                () -> mTeleopAutoMode.runAction(new ElevatorAction("L3")));         //Same as L2
        mXboxController1.y(m_loop).rising().ifHigh(
                () -> mTeleopAutoMode.runAction(new ElevatorAction("L4")));         //Pm shoot button, move pivot to barge and shoot
        mXboxController1.leftBumper(m_loop).rising().ifHigh(
                () -> mTeleopAutoMode.runAction(new SnapToTag("LEFTBAR", "R")));
        mXboxController1.rightBumper(m_loop).rising().ifHigh(
                () -> mTeleopAutoMode.runAction(new SnapToTag("RIGHTBAR", "R")));
        mXboxController1.rightTrigger(0.5, m_loop).ifHigh(
                () -> mTeleopAutoMode.runAction(new CoralShootAction()));
        mXboxController1.leftTrigger(0.5, m_loop).rising().ifHigh(
                () -> mTeleopAutoMode.runAction(new AlgaePivotAction("Intake")));
        mXboxController1.leftTrigger(0.5, m_loop).falling().ifHigh(
                () -> mTeleopAutoMode.runAction(new AlgaePivotAction("Ground")));
                //() -> mTeleopAutoMode.runAction(new SnapToTag("CS", "CS")));
        mXboxController1.start(m_loop).ifHigh(
                () -> isFieldRelative = !isFieldRelative);
        mXboxController1.back(m_loop).ifHigh(
                () -> mTeleopAutoMode.runAction(new SnapToTag("CS","CS")));
        
    }

    public void processKeyCommand() {
        if (mTeleopAutoMode == null)
            return;

        m_loop.poll();

        double translationVal = -MathUtil.applyDeadband(m_JoyStick.getRawAxis(translationAxis), Constants.stickDeadband)
                * Constants.SwerveConstants.maxSpeed;
        double strafeVal = -MathUtil.applyDeadband(m_JoyStick.getRawAxis(strafeAxis), Constants.stickDeadband)
                * Constants.SwerveConstants.maxSpeed;
        double rotationVal = -MathUtil.applyDeadband(m_JoyStick.getRawAxis(rotationAxis), Constants.stickDeadband)
                * Constants.Swerve.maxAngularVelocity;

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

        Twist2d velocity = RobotState.getInstance().getMeasuredVelocity();

        // if (m_VisionDevices.inRange() &&
        //         Math.sqrt(Math.pow(velocity.dx, 2) + Math.pow(velocity.dy, 2)) < 1) {
        if (m_VisionDevices.inRange()) {
            mXboxController1.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
            m_LED.green();
        } else {
            mXboxController1.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            m_LED.red();
        }

    }
}
