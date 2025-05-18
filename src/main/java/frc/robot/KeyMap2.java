package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

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
import frc.robot.lib.controllers.XboxBinder;
import frc.robot.lib.loops.Looper;
import frc.robot.lib.util.Util;
import static frc.robot.lib.controllers.XboxBinder.*;

/**
 * A class that contains the controls for the robot.
 */
public class KeyMap2 {
    private XboxController mXboxController1 = null;
    private ActionExecutor mActionExecutor = null;
    private Drive mSwerveDrive = null;
    private VisionDeviceManager mVisionDevices = null;
    private LED mLED = null;

    private XboxBinder mBinder = null;

    public KeyMap2(
        XboxController xboxController1, 
		Looper enabledLooper,
        ActionExecutor actionExecutor
    ) {
        mXboxController1 = xboxController1;
        mActionExecutor = actionExecutor;
        mSwerveDrive = Drive.getInstance();
        mVisionDevices = VisionDeviceManager.getInstance();
        mLED = LED.getInstance();

        mBinder = new XboxBinder(mXboxController1, enabledLooper, actionExecutor);
        
        //XYAB
        mBinder.bind(Buttons.A, When.CLICKED, new EmptyAction());
        mBinder.bind(Buttons.B, When.CLICKED, new EmptyAction());
        mBinder.bind(Buttons.X, When.CLICKED, new EmptyAction());
        mBinder.bind(Buttons.Y, When.CLICKED, new EmptyAction());

        //Bumper
        mBinder.bind(Buttons.LEFT_BUMPER, When.CLICKED, new SnapToTag("CENTER", "ANY"));
        mBinder.bind(Buttons.RIGHT_BUMPER, When.CLICKED, new EmptyAction());

        //Triggers
        mBinder.bindAxisAsButton(Axes.LEFT_TRIGGER, 0.5, When.CLICKED, new SnapToTag("CENTER", "ANY"));
        mBinder.bindAxisAsButton(Axes.RIGHT_TRIGGER, 0.5, When.CLICKED, new EmptyAction());

        //Joysticks
		mBinder.bindAxes((List<Double> values) -> {
			double x = values.get(0);
			double y = values.get(0);
			double z = values.get(0); 
			Optional<Alliance> ally = RobotState.getAlliance();
			if (ally.isPresent()) {
				if (ally.get() == Alliance.Red) {
					mSwerveDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
                        y * Constants.Swerve.MAX_SPEED, x * Constants.Swerve.MAX_SPEED, z * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                        Util.robotToFieldRelative(mSwerveDrive.getHeading(), true)));
				} else {
					mSwerveDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
                        - y * Constants.Swerve.MAX_SPEED, - x * Constants.Swerve.MAX_SPEED, z * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                        Util.robotToFieldRelative(mSwerveDrive.getHeading(), true)));
				}
			}
        }, Axes.LEFT_JOYSTICK_X, Axes.LEFT_JOYSTICK_Y, Axes.RIGHT_JOYSTICK_X);

		mBinder.start();
    }

    public void processKeyCommand() {
        if (mActionExecutor == null)
            return;
    }
}
