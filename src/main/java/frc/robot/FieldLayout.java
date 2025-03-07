package frc.robot;

import java.io.IOException;
import java.util.HashMap;
import java.util.Optional;

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
    public static HashMap<String, Translation2d> offsets = new HashMap<>();

	public static HashMap<String, int[]> presets = new HashMap<>();


	public static final double kApriltagWidth = Units.inchesToMeters(6.50);
	public static final AprilTagFieldLayout kTagMap;

	static {
		try {	
			offsets.put("LEFTBAR",
				new Translation2d(Constants.Swerve.trackWidth/2 + 0.189, -0.166));
			offsets.put("RIGHTBAR",
				new Translation2d(Constants.Swerve.trackWidth/2 + 0.189, 0.166));
			offsets.put("CENTER",
				new Translation2d(Constants.Swerve.trackWidth/2 + 0.189,0.0));
			offsets.put("CS",
				new Translation2d(-Constants.Swerve.trackWidth/2 - 0.20, 0));
			offsets.put("HANG",
				new Translation2d(0,0));
			
			presets.put("CS", new int[] {1, 2, 12, 13});
			presets.put("R", new int[] {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22});
			presets.put("P", new int[] {3, 16});
			presets.put("ANY", new int[] {});
			presets.put("BARGE", new int[] {4, 5, 14, 15});
			presets.put("NOBARGE", new int[] {-4, -5, -14, -15});
			kTagMap = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
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

	public static AprilTag getClosestTag(Translation2d robot_position) {
		return getClosestTag(robot_position, presets.get("NOBARGE"));
	}
	/**
	 * 
	 * @param robot_position The current robot position
	 * @param ids Positive for whitelist, negative for blacklist
	 * @return The closest apriltag on the field
	 */
	public static AprilTag getClosestTag(Translation2d robot_position, int... ids) {
		AprilTag closest_tag = null;
		double closest_distance = Double.MAX_VALUE;

		for (AprilTag tag : kTagMap.getTags()) {
			double distance = robot_position.getDistance(tag.pose.getTranslation().toTranslation2d());

			for (int num : ids) {
				int absNum = Math.abs(num);

				if (num == absNum) {
					distance -= 1000000000; //hopefully there is no situation where this is insufficient
				} else if (num == tag.ID) {
					distance = Double.POSITIVE_INFINITY;
				}
			}

			if (distance < closest_distance) {
				closest_tag = tag;
				closest_distance = distance;
			}
		}

		return closest_tag;
	}

	public static AprilTag getClosestTag(Translation2d robot_position, String preset) {
		return getClosestTag(robot_position, presets.get(preset));
	}
}