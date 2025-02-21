package frc.robot.teleop;
import java.util.List;

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
    private TeleopAutoMode mTeleopAutoMode = null;
    private boolean prev_left_trigger = false;
    private boolean prev_right_trigger = false;
    private int prevPOV = -1;

    // constructor
    public Controller(XboxController xboxController, TeleopAutoMode teleopAutoMode) {
        mXboxController1 = xboxController;
        mTeleopAutoMode = teleopAutoMode;
    }

    // key event handling in manual mode periodic 
    public void processKeyCommand() {
        //System.out.println("updated");
        if ( mTeleopAutoMode==null) return;
        
        //abort
        if (mXboxController1.getRawButton(7)) {
            System.out.println("attempted to abort actions");
            mTeleopAutoMode.abort();
            return;
        }

        //elevator
        if (mXboxController1.getYButtonPressed()) {
            mTeleopAutoMode.runAction(new ElevatorAction("L4"));
        }
        if (mXboxController1.getAButtonPressed()) {
            mTeleopAutoMode.runAction(new ElevatorAction("Ground"));
        }
        
        if (mXboxController1.getBButtonPressed()) {
            //System.out.println("L2 Called");
            mTeleopAutoMode.runAction(new ElevatorAction("L2"));
        }
        if (mXboxController1.getXButtonPressed()) {
            mTeleopAutoMode.runAction(new ElevatorAction("L3"));
        }

        //algae
        if (mXboxController1.getLeftTriggerAxis()> 0.5) {
            if (!prev_left_trigger) {
                prev_left_trigger = true;
                mTeleopAutoMode.runAction(new AlgaeShooterAction());
            }
        } else {
            prev_left_trigger = false;
        }
        if (mXboxController1.getRightTriggerAxis()>0.5) {
        //     if (!prev_right_trigger) {
        //         mTeleopAutoMode.runAction(new CoralShootAction());
        //     }
        // } else {
        //     prev_right_trigger = false;
        }

        //Coral
        if (mXboxController1.getLeftBumperButtonPressed()) {
            mTeleopAutoMode.runAction(new AlgaeShooterAction());
            /*int tag = FieldLayout.getClosestTag(RobotState.getInstance().getLatestFieldToVehicle().getTranslation());
            boolean isL3 = false;
            for (int num : new int[] {18, 20, 22, 7, 9, 11}) {
                if (num == tag) {
                    isL3 = true;
                }
            }*/
            //mTeleopAutoMode.runAction(new SeriesAction(
                //new ElevatorAction(isL3 ? 3 : 2),
                //new SnapToTag(2),
               // new AlgaeIntakeAction()
           // ));
        }
        if (mXboxController1.getRightBumperButtonPressed()) {
            
            mTeleopAutoMode.runAction(new CoralShootAction());
        }

        //Drive train
        if (mXboxController1.getPOV() == 0 && prevPOV != 0) {
            //mTeleopAutoMode.runAction(new SnapToTag(2));
        }
        if (mXboxController1.getPOV() == 90 && prevPOV != 90) {
            //mTeleopAutoMode.runAction(new SnapToTag(1));
        }
        if (mXboxController1.getPOV() == 270 && prevPOV != 270) {
            //mTeleopAutoMode.runAction(new SnapToTag(0));
        }

        //prevPOV = mXboxController1.getPOV();
    }
}
