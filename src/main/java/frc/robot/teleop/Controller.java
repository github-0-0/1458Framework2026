package frc.robot.teleop;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.FieldLayout;
import frc.robot.RobotState;
import frc.robot.autos.actions.*;
import frc.robot.autos.modes.TeleopAutoMode;
/**
 * A class that contains the controls for the robot.
 */
public class Controller {
    private XboxController mXboxController1 = null;
    private XboxController mXboxController2 = null;
    private TeleopAutoMode mTeleopAutoMode = null;
    private boolean POVReset = true;
    private boolean lTriggerReset = true;
    private boolean rTriggerReset = true;

    // constructor
    public Controller(XboxController xboxController1, XboxController xboxController2, TeleopAutoMode teleopAutoMode) {
        mXboxController1 = xboxController1;
        mXboxController2 = xboxController2;
        mTeleopAutoMode = teleopAutoMode;
    }

    // key event handling in manual mode periodic 
    public void processKeyCommand() {
        if (mTeleopAutoMode == null) return;
        
        //PAGE BUTTON: abort
        if (mXboxController1.getRawButton(7)) {
            System.out.println("attempted to abort actions");
            mTeleopAutoMode.abort();
            return;
        }

        //XYAB: elevator
        if (mXboxController1.getYButtonPressed()) {
            mTeleopAutoMode.runAction(new ElevatorAction("L4"));
        } else if (mXboxController1.getAButtonPressed()) {
            mTeleopAutoMode.runAction(new ElevatorAction("Ground"));
        } else if (mXboxController1.getBButtonPressed()) {
            mTeleopAutoMode.runAction(new ElevatorAction("L2"));
        } else if (mXboxController1.getXButtonPressed()) {
            mTeleopAutoMode.runAction(new ElevatorAction("L3"));
        }

        //POVs: reserved for nudge
        if (mXboxController1.getPOV() == 0 && POVReset) {
            POVReset = false;
        } else if (mXboxController1.getPOV() == 90 && POVReset) {
            POVReset = false;
        } else if (mXboxController1.getPOV() == 180 && POVReset) {
            POVReset = false;
        } else if (mXboxController1.getPOV() == 270 && POVReset) {
            POVReset = false;
        } else {
            POVReset = true;
        }
        
        //BUMPERS: snap
        if (mXboxController1.getLeftBumperButtonPressed()) {
            mTeleopAutoMode.runAction(new SnapToTag("LEFTBAR"));
        } else if(mXboxController1.getRightBumperButtonPressed()) {
            mTeleopAutoMode.runAction(new SnapToTag("RIGHTBAR"));
        }

        //TRIGGER AXES: shooting
        if (mXboxController1.getLeftTriggerAxis()> 0.5 && lTriggerReset) {
            lTriggerReset = false;
        } else {
            lTriggerReset = true;
        }

        if (mXboxController1.getRightTriggerAxis()>0.5 && rTriggerReset) {
            mTeleopAutoMode.runAction(new CoralShootAction());
            rTriggerReset = false;
        } else {
            rTriggerReset = true;
        }
    }
}