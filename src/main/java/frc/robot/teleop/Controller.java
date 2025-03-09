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
    private int snapTo = 0;

    
    // constructor
    public Controller(XboxController xboxController1, XboxController xboxController2, TeleopAutoMode teleopAutoMode) {
        mXboxController1 = xboxController1;
        mXboxController2 = xboxController2;
        mTeleopAutoMode = teleopAutoMode;
    }

    // key event handling in manual mode periodic 


    public void processKeyCommand() {
        //System.out.println("updated");
        if ( mTeleopAutoMode==null) return;
        
        //abort
        // if (mXboxController1.getRawButton(7)) {
        //     System.out.println("attempted to abort actions");
        //     mTeleopAutoMode.abort();
        //     return;
        // }

        //elevator
        if (mXboxController1.getYButtonPressed()) {
            //Elevator.getInstance().runElevatorRaw(0.1);
            mTeleopAutoMode.runAction(new ElevatorAction("L4"));
            // mTeleopAutoMode.runAction(new SeriesAction(new ElevatorAction("L4"), new SnapToTag(snapTo), new CoralShootAction(), new ElevatorAction("L3")));
        }
        else if (mXboxController1.getAButtonPressed()) {
//            Elevator.getInstance().runElevatorRaw(-0.1);
            mTeleopAutoMode.runAction(new ElevatorAction("Ground"));
            //mTeleopAutoMode.runAction(new SeriesAction(new ElevatorAction("Ground"), new SnapToTag(2)));
        }
        else if (mXboxController1.getBButtonPressed()) {
            //System.out.println("L2 Called");
            mTeleopAutoMode.runAction(new ElevatorAction("L2"));
            // mTeleopAutoMode.runAction(new SeriesAction(new ElevatorAction("L2"), new SnapToTag(snapTo), new CoralShootAction()));
        }
        else if (mXboxController1.getXButtonPressed()) {
            mTeleopAutoMode.runAction(new ElevatorAction("L3"));
            // mTeleopAutoMode.runAction(new SeriesAction(new ElevatorAction("L3"), new SnapToTag(snapTo), new CoralShootAction(), new ElevatorAction("L3")));
        }
        else if (mXboxController1.getPOV() == 0 && POVReset) {
            POVReset = false;
           // mTeleopAutoMode.runAction(new SeriesAction(new ElevatorAction("A2"), new SnapToTag(2), new AlgaeAction("Intake")));
        }
        else if (mXboxController1.getPOV() == 180 && POVReset) {
            POVReset = false;
            //mTeleopAutoMode.runAction(new SeriesAction(new ElevatorAction("A1"), new SnapToTag(2), new AlgaeAction("Intake")));
        }
        else if (mXboxController1.getRightBumperButtonPressed()) {
            // mTeleopAutoMode.runAction(new CoralShootAction());
            // mTeleopAutoMode.runAction(new SeriesAction(new ElevatorAction("AP"), new SnapToTag(2), new AlgaeAction("Shoot"), new ElevatorAction("L2")));
            // mTeleopAutoMode.runAction(new AlgaeAction("Shoot"));
            // System.out.println("right  bumper fired");
            mTeleopAutoMode.runAction(new SnapToTag(1));
        }
        else if(mXboxController1.getLeftBumperButtonPressed()) {
            // mTeleopAutoMode.runAction(new AlgaeAction("Intake"));
            // mTeleopAutoMode.runAction(new SeriesAction(new ElevatorAction("Ground"), new AlgaeAction("Intake"), new ElevatorAction("L2")));
            // System.out.println("Left bumper fired");
            mTeleopAutoMode.runAction(new SnapToTag(0));
        }
        else if (mXboxController1.getLeftTriggerAxis()> 0.5) {
            // System.out.println("Left trigger fired");
            
        }
        else if (mXboxController1.getRightTriggerAxis()>0.5) {
            // System.out.println("right trigger fired");
            mTeleopAutoMode.runAction(new CoralShootAction());
        }
        else {
            POVReset = true;
        }

        // if(mXboxController2.getPOV() == 90) {
        //     mTeleopAutoMode.runAction(new ElevatorAction("A1"));
        // }
        // if(mXboxController2.getPOV() == 270) {
        //     mTeleopAutoMode.runAction(new ElevatorAction("A2"));
        // }

        //algae
        /*
        if (mXboxController1.getLeftTriggerAxis()> 0.5) {
           
            if (mCurrActionAlgaeShoot==null) {
                mCurrActionAlgaeShoot = new AlgaeShooterAction();
                mTeleopAutoMode.runAction(mCurrActionAlgaeShoot);
            }else{
                if (mCurrActionAlgaeShoot.isFinished()){
                    mCurrActionAlgaeShoot=null;
                }
            }
        } 
        if (mXboxController1.getRightTriggerAxis()>0.5) {
             if (mCurrActionAlgaeIntake==null) {
                mCurrActionAlgaeIntake = new AlgaeIntakeAction();
                mTeleopAutoMode.runAction(mCurrActionAlgaeIntake);
             } else {
                if (mCurrActionAlgaeIntake.isFinished()){
                    mCurrActionAlgaeIntake=null;
                }
             }
        }
              */

        //Coral
        if (mXboxController1.getLeftBumperButtonPressed()) {
            //mTeleopAutoMode.runAction(new AlgaeShooterAction());
            //mTeleopAutoMode.runAction(new SeriesAction(new ElevatorAction("L3"), new CoralAction("Shoot")));
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
        // if(mXboxController2.getBButton()) {
        //     mTeleopAutoMode.runAction(new ElevatorAction("L3"));
        // }
        // //hang            
        // if (mXboxController1.getPOV()==0) {
        //     Hang.getInstance().setMotor(0.3);
        // } else if (mXboxController1.getPOV()==180) {
        //     Hang.getInstance().setMotor(-0.3);
        // } else {
        //     Hang.getInstance().setMotor(-0);
        // }


        //snap2tag
        /* 
        if (mXboxController1.getPOV() == 0 && prevPOV != 0) {
            mTeleopAutoMode.runAction(new SnapToTag(2));
            
        }
        if (mXboxController1.getPOV() == 90 && prevPOV != 90) {
            mTeleopAutoMode.runAction(new SnapToTag(1));
        }
        if (mXboxController1.getPOV() == 270 && prevPOV != 270) {
            mTeleopAutoMode.runAction(new SnapToTag(0));
        }*/

        
    }
}
