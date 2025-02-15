package frc.robot;

import java.io.IOException;
import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in
 * Meters</b> <br>
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldLayout {
	public static double kFieldLength = Units.inchesToMeters(651.223);
	public static double kFieldWidth = Units.inchesToMeters(323.277);
	public static int closestTag = -1;
    public static Translation2d[] offsets = {
		new Translation2d(0.85/2,-0.18),
		new Translation2d(0.85/2, 0.18),
		new Translation2d(0.85/2,0),
		new Translation2d(-0.85/2,-0.18),
		new Translation2d(-0.85/2,0),
		new Translation2d(-0.85/2,0.18),
	};

	public static final double kApriltagWidth = Units.inchesToMeters(6.50);
	public static final AprilTagFieldLayout kTagMap;

	static {
		try {
			kTagMap = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	public static Pose2d handleAllianceFlip(Pose2d blue_pose, boolean is_red_alliance) {
		if (is_red_alliance) {
            blue_pose = new Pose2d(
                new Translation2d((kFieldLength / 2.0) + ((kFieldLength / 2.0) - blue_pose.getX()), blue_pose.getY()), 
                new Rotation2d(-blue_pose.getRotation().getCos(), blue_pose.getRotation().getSin())
            );
		}
		return blue_pose;
	}

	public static Translation2d handleAllianceFlip(Translation2d blue_translation, boolean is_red_alliance) {
		if (is_red_alliance) {
            blue_translation = new Translation2d((kFieldLength / 2.0) + ((kFieldLength / 2.0) - blue_translation.getX()), blue_translation.getY());
		}
		return blue_translation;
	}

	public static Rotation2d handleAllianceFlip(Rotation2d blue_rotation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_rotation = new Rotation2d(-blue_rotation.getCos(), blue_rotation.getSin());
		}
		return blue_rotation;
	}

	public static double distanceFromAllianceWall(double x_coordinate, boolean is_red_alliance) {
		if (is_red_alliance) {
			return kFieldLength - x_coordinate;
		}
		return x_coordinate;
	}

	public static Pose3d getClosestTagPos(Translation2d robot_position) {
		Pose3d closest_tag = new Pose3d();
		double closest_distance = Double.MAX_VALUE;
		for (AprilTag tag : kTagMap.getTags()) {
			double distance = robot_position.getDistance(tag.pose.getTranslation().toTranslation2d());
			if (distance < closest_distance) {
				closest_tag = tag.pose;
				closest_distance = distance;
			}
		}
		return closest_tag;
	}

	public static int getClosestTag(Translation2d robot_position) {
		int closest_tag = -1;
		double closest_distance = Double.MAX_VALUE;
		for (AprilTag tag : kTagMap.getTags()) {
			double distance = robot_position.getDistance(tag.pose.getTranslation().toTranslation2d());
			if (distance < closest_distance) {
				closest_tag = tag.ID;
				closest_distance = distance;
			}
		}
		closestTag = closest_tag;
		return closest_tag;
	}
}