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

public class PIDHolonomicDriveController implements DriveController {
    private final PIDV xController;
    private final PIDV yController;
    private final ProfiledPIDV thetaController;
    private double translationKa;

    private TrajectoryIterator trajectory;
    private Pose2d currentPose;
    private ChassisSpeeds currentSpeeds;

    private boolean enabled = true;

    private double prevTime = 0.0;

    private RobotState mRobotState = RobotState.getInstance();

    public PIDHolonomicDriveController(PIDConstants translationConstants, PIDConstants rotationConstants, TrapezoidProfile.Constraints thetaConstraints) {
        xController = new PIDV(
                translationConstants.kP, translationConstants.kI, translationConstants.kD, 0.02);
        yController = new PIDV(
                translationConstants.kP, translationConstants.kI, translationConstants.kD, 0.02);

        xController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);
        yController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);

        thetaController = new ProfiledPIDV(
                rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, thetaConstraints);
        thetaController.setIntegratorRange(-rotationConstants.iZone, rotationConstants.iZone);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void setTrajectory(TrajectoryIterator trajectory) {
        this.trajectory = trajectory;
        prevTime = Timer.getFPGATimestamp();
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
            thetaController.reset(currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
            prevTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public ChassisSpeeds calculate() {
        setRobotState(mRobotState.getLatestFieldToVehicle(), mRobotState.getSmoothedVelocity());
        if (trajectory == null || currentPose == null || currentSpeeds == null || trajectory.isDone()) {
            return new ChassisSpeeds(0, 0, 0);
        }

        Trajectory.State targetState = trajectory.getState();
        trajectory.advance(Timer.getFPGATimestamp() - prevTime); // Move to next state internally

        double vxFF = targetState.velocityMetersPerSecond * targetState.poseMeters.getRotation().getCos();
        double vyFF = targetState.velocityMetersPerSecond * targetState.poseMeters.getRotation().getSin();

        // double xAccelFF = targetState.accelerationMetersPerSecondSq * targetState.poseMeters.getRotation().getCos();
        // double yAccelFF = targetState.accelerationMetersPerSecondSq * targetState.poseMeters.getRotation().getSin();

        // Optional: curvature-based angular acceleration contribution
        // double angularAccel = targetState.curvatureRadPerMeter * targetState.velocityMetersPerSecond * targetState.velocityMetersPerSecond;
        // xAccelFF += -angularAccel * targetState.poseMeters.getRotation().getSin();
        // yAccelFF += angularAccel * targetState.poseMeters.getRotation().getCos();

        double xFeedback = xController.calculate(
                currentPose.getX(), currentSpeeds.vxMetersPerSecond, targetState.poseMeters.getX(), vxFF);
        double yFeedback = yController.calculate(
                currentPose.getY(), currentSpeeds.vyMetersPerSecond, targetState.poseMeters.getY(), vyFF);

        double thetaFeedback = thetaController.calculate(
                currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond,
                new TrapezoidProfile.State(targetState.poseMeters.getRotation().getRadians(), 0), new TrapezoidProfile.Constraints(Constants.Swerve.maxAngularVelocity, Constants.Swerve.kMaxAngularAcceleration));

        double rotation = thetaFeedback + thetaController.getSetpoint().velocity;
        
        prevTime = Timer.getFPGATimestamp();

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                vxFF + xFeedback, // + xAccelFF * translationKa,
                vyFF + yFeedback, // + yAccelFF * translationKa,
                rotation,
                currentPose.getRotation());
    }

    @Override
    public boolean isDone() {
        if (trajectory == null || trajectory.isDone()) {
            System.out.println("Done with trajectory, error: " + Math.hypot(xController.getPositionError(), yController.getPositionError()));
            return true;
        }
        return false;
    }
}
