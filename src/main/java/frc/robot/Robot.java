// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Loops.Looper;
//import frc.robot.controlboard.ControlBoard;
//import frc.robot.controlboard.DriverControls;
import frc.robot.subsystems.DummySubsystem;
import frc.robot.subsystems.SubsystemManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // @-@ new objects from Framework25
  
   public final SubsystemManager m_SubsystemManager = SubsystemManager.getInstance();
   //private final ControlBoard mControlBoard = ControlBoard.getInstance();
	 //private final DriverControls mDriverControls = new DriverControls();

   private final Looper m_EnabledLooper = new Looper();
   private final Looper m_DisabledLooper = new Looper();
   private DummySubsystem m_ExampleSubsystem;


  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  
  private Command m_autonomousCommand;

  private RobotContainer25 m_robotContainer;

  private Field2d m_robotStateField;
  private Pose2d m_robotStatePose;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer25();
    

    for (int port = 5800; port <= 5809; port++) {
      edu.wpi.first.net.PortForwarder.add(port, "limelight-left.local", port);
      edu.wpi.first.net.PortForwarder.add(port, "limelight-right.local", port);
      edu.wpi.first.net.PortForwarder.add(port, "limelight-front.local", port);
      edu.wpi.first.net.PortForwarder.add(port, "limelight-back.local", port);
    }
    
    m_robotStateField = new Field2d();
    m_robotStatePose = new Pose2d();

    m_robotStateField.setRobotPose(m_robotStatePose);
    
    SmartDashboard.putData("Robot State", m_robotStateField);

    CanBridge.runTCP();
    //subsystems and loop framework init code move to RobotContainer25 class  
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    m_robotStatePose = RobotState.getInstance().getLatestFieldToVehicle();
    m_robotStateField.setRobotPose(m_robotStatePose);

    SmartDashboard.putData("Robot State", m_robotStateField);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.initDisabledMode();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    //init auto mode
    m_robotContainer.initAutoMode();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //mControlBoard.update(); //dc:TODO: revisit controlboard class late;  
    //initialize container for teleop mode 
    m_robotContainer.initManualMode();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.manualModePeriodic();  //run the manual mode loop
    m_robotContainer.updateLimeLightData();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.initTestMode();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_robotContainer.testModePeriodic();
    m_robotContainer.updateLimeLightData();
  }
}
