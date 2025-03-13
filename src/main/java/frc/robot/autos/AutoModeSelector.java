package frc.robot.autos;

import frc.robot.autos.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class AutoModeSelector {
	public enum DesiredMode {
		DO_NOTHING,
		LEFTSIDE,
		RIGHTSIDE,
		CENTER,
		TESTAUTOMODE3,
	}

	private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

	private Optional<AutoModeBase> mAutoMode = Optional.empty();

	private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();

	public AutoModeSelector() {
		mModeChooser.addOption("DoNothing", DesiredMode.DO_NOTHING);
		mModeChooser.addOption("LeftSide", DesiredMode.LEFTSIDE);
		mModeChooser.addOption("RightSide", DesiredMode.RIGHTSIDE);
		mModeChooser.addOption("Center", DesiredMode.CENTER);
		mModeChooser.addOption("AutoMode3", DesiredMode.TESTAUTOMODE3);
		mModeChooser.setDefaultOption("AutoMode3", DesiredMode.TESTAUTOMODE3);
		SmartDashboard.putData("Auto Mode", mModeChooser);
	}

	public void updateModeCreator(boolean force_regen) {
		DesiredMode desiredMode = mModeChooser.getSelected();

		if (desiredMode == null) {
			desiredMode = DesiredMode.DO_NOTHING;
		}
		if (mCachedDesiredMode != desiredMode
		|| force_regen) {
			System.out.println("Auto selection changed, updating creator: desiredMode-> " + desiredMode.name());
			
			mAutoMode = getAutoModeForParams(desiredMode);
			}
		mCachedDesiredMode = desiredMode;
	}

	// public void forceModeTo(boolean force_regen){
	// 	DesiredMode desiredMode = DesiredMode.TESTPATHMODE;

	// 	if (desiredMode == null) {
	// 		desiredMode = DesiredMode.DO_NOTHING;
	// 	}
	// 	if (mCachedDesiredMode != desiredMode
	// 	|| force_regen) {
	// 		System.out.println("Auto selection changed, updating creator: desiredMode-> " + desiredMode.name());
			
	// 		mAutoMode = getAutoModeForParams(desiredMode);
	// 		}
	// 	mCachedDesiredMode = desiredMode;
	// }

	private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode ){
		switch (mode) {
			case DO_NOTHING:
				return Optional.of(new DoNothingMode());
			case LEFTSIDE:
				return Optional.of(new AutoModeLeftSide());
			case RIGHTSIDE:
				return Optional.of(new AutoModeRightSide());
			case CENTER:
				return Optional.of(new AutoModeCenter());
			case TESTAUTOMODE3:
				return Optional.of(new TestAutoMode3());
			default:
				System.out.println("ERROR: unexpected auto mode: " + mode);
				break;
		}

		System.err.println("No valid auto mode found for  " + mode);
		return Optional.empty();
	}

	public static SendableChooser<DesiredMode> getModeChooser() {
		return mModeChooser;
	}

	public DesiredMode getDesiredAutomode() {
		return mCachedDesiredMode;
	}

	public void reset() {
		mAutoMode = Optional.empty();
		mCachedDesiredMode = null;
	}

	public void outputToSmartDashboard() {
		SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
	}

	public Optional<AutoModeBase> getAutoMode() {
		if (!mAutoMode.isPresent()) {
			return Optional.empty();
		}
		return mAutoMode;
	}

	public boolean isDriveByCamera() {
		return mCachedDesiredMode == DesiredMode.DO_NOTHING;
	}
}
