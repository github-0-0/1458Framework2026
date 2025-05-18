package frc.robot.lib.swerve;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.lib.control.PIDV;
import frc.robot.lib.control.ProfiledPIDV;
import frc.robot.lib.trajectory.TrajectoryIterator;
import frc.robot.lib.util.Stopwatch;
import frc.robot.lib.util.Util;

public class PIDHolonomicDriveController implements DriveController {
    private final PIDV mXController;
    private final PIDV mYController;
    private final ProfiledPIDV mThetaController;
    private double translationKa;

    private TrajectoryIterator trajectory;
    private Pose2d currentPose;
    private ChassisSpeeds currentSpeeds;

    private boolean enabled = true;

    private Stopwatch mStopwatch = null;

    public PIDHolonomicDriveController(PIDConstants translationConstants, PIDConstants rotationConstants, TrapezoidProfile.Constraints thetaConstraints) {
        mXController = new PIDV(
                translationConstants.kP, translationConstants.kI, translationConstants.kD, Constants.LOOPER_DT);
        mYController = new PIDV(
                translationConstants.kP, translationConstants.kI, translationConstants.kD, Constants.LOOPER_DT);

        mXController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);
        mYController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);

        mThetaController = new ProfiledPIDV(
                rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, thetaConstraints);
        mThetaController.setIntegratorRange(-rotationConstants.iZone, rotationConstants.iZone);
        mThetaController.enableContinuousInput(-Math.PI, Math.PI);
        mStopwatch = new Stopwatch();
    }

    @Override
    public void setTrajectory(TrajectoryIterator trajectory) {
        this.trajectory = trajectory;
        mStopwatch.reset();
        this.trajectory.visualizeTrajectory();
    }

    public void setRobotState(Pose2d pose, Twist2d speeds) {
        this.currentPose = pose;
        this.currentSpeeds = new ChassisSpeeds(speeds.dx, speeds.dy, speeds.dtheta);
    }

    @Override
    public void reset() {
        if (trajectory != null && trajectory.getInitialState() != null) {
            trajectory.advance(Double.MIN_VALUE);
            mThetaController.reset(currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
            mStopwatch.reset();
        }
    }

    @Override
    public ChassisSpeeds calculate() {
        setRobotState(RobotState.getLatestFieldToVehicle(), RobotState.getSmoothedVelocity());
        if (trajectory == null || currentPose == null || currentSpeeds == null || trajectory.isDone()) {
            return new ChassisSpeeds();
        }

        Trajectory.State targetState = trajectory.getState();
        trajectory.advance(mStopwatch.getDeltaTime()); // Move to next state internally
        mStopwatch.update();

        double vxFF = targetState.velocityMetersPerSecond * targetState.poseMeters.getRotation().getCos();
        double vyFF = targetState.velocityMetersPerSecond * targetState.poseMeters.getRotation().getSin();

        double xAccelFF = Util.deadBand(
                targetState.accelerationMetersPerSecondSq * targetState.poseMeters.getRotation().getCos(),    
                Constants.Swerve.MAX_ACCELERATION * 0.5);
        double yAccelFF = Util.deadBand(
                targetState.accelerationMetersPerSecondSq 
                * targetState.poseMeters.getRotation().getSin(),
                Constants.Swerve.MAX_ACCELERATION * 0.5);

        // Optional: curvature-based angular acceleration contribution
        double angularAccel = Util.deadBand(
                targetState.curvatureRadPerMeter * targetState.velocityMetersPerSecond * targetState.velocityMetersPerSecond,
                Constants.Swerve.MAX_ANGULAR_ACCELERATION * 0.5);
        xAccelFF += -angularAccel * targetState.poseMeters.getRotation().getSin();
        yAccelFF += angularAccel * targetState.poseMeters.getRotation().getCos();

        double xFeedback = mXController.calculate(
                currentPose.getX(), currentSpeeds.vxMetersPerSecond, targetState.poseMeters.getX(), vxFF);
        double yFeedback = mYController.calculate(
                currentPose.getY(), currentSpeeds.vyMetersPerSecond, targetState.poseMeters.getY(), vyFF);

        double thetaFeedback = mThetaController.calculate(
                currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond,
                new TrapezoidProfile.State(targetState.poseMeters.getRotation().getRadians(), 0), new TrapezoidProfile.Constraints(Constants.Swerve.MAX_ANGULAR_VELOCITY, Constants.Swerve.MAX_ANGULAR_ACCELERATION));

        double rotation = thetaFeedback + mThetaController.getSetpoint().velocity;

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                vxFF + xFeedback + xAccelFF * translationKa,
                vyFF + yFeedback + yAccelFF * translationKa,
                rotation,
                currentPose.getRotation());
    }

    @Override
    public boolean isDone() {
        if (trajectory == null || trajectory.isDone()) {
            System.out.println("Done with trajectory, error: " + Math.hypot(mXController.getPositionError(), mYController.getPositionError()));
            return true;
        }
        return false;
    }
}
