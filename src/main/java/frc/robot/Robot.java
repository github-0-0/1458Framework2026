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
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SubsystemManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	private RobotContainer m_robotContainer;

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
		m_robotContainer = new RobotContainer();
		

		for (int port = 5800; port <= 5809; port++) {
			edu.wpi.first.net.PortForwarder.add(port, "limelight-left.local", port);
			edu.wpi.first.net.PortForwarder.add(port, "limelight-right.local", port);
			edu.wpi.first.net.PortForwarder.add(port, "limelight-front.local", port);
			edu.wpi.first.net.PortForwarder.add(port, "limelight-back.local", port);
		}
		m_robotStateField = new Field2d();
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
		//SmartDashboard.putNumber("Algae Shooter", Laser.getMeasurementAlgaeShooter());
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
	public void disabledPeriodic() {
		m_robotContainer.disabledPeriodicMode();
	}

	/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
	@Override
	public void autonomousInit() {
		//init auto mode
		m_robotContainer.initAutoMode();
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		//initialize container for teleop mode 
		m_robotContainer.initManualMode();

	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		m_robotContainer.manualModePeriodic();  //run the manual mode loop
	}

	@Override
	public void testInit() {
		m_robotContainer.initTestMode();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
		m_robotContainer.testModePeriodic();
	}
}
