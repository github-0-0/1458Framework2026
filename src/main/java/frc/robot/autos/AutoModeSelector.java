package frc.robot.autos;

import frc.robot.autos.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class AutoModeSelector {
	public enum DesiredMode {
		DO_NOTHING,
		TESTPATHMODE,
		TESTAUTOMODE,
		TESTAUTOMODE2,
		TESTAUTOMODE3,
		LEFTCORAL,
		LMIDDLECORAL,
		RLMIDDLE,
		RRMIDDLE,
		LRMIDDLE,
		LLMIDDLE,
		L4AROUND
	}

	private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

	private Optional<AutoModeBase> mAutoMode = Optional.empty();

	private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();

	public AutoModeSelector() {
		mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
		mModeChooser.addOption("mode to test paths", DesiredMode.TESTPATHMODE);
		mModeChooser.addOption("non working 1", DesiredMode.TESTAUTOMODE);
		mModeChooser.addOption("non working 2", DesiredMode.TESTAUTOMODE2);
		mModeChooser.addOption("mode to test autos", DesiredMode.TESTAUTOMODE3);
		mModeChooser.addOption("Left Coral", DesiredMode.LEFTCORAL);
		mModeChooser.addOption("LMiddle Coral", DesiredMode.LMIDDLECORAL);
		mModeChooser.addOption("RLMiddle", DesiredMode.RLMIDDLE);
		mModeChooser.addOption("RRMiddle!", DesiredMode.RRMIDDLE);
		mModeChooser.addOption("LRMiddle", DesiredMode.LRMIDDLE);
		mModeChooser.addOption("L4Around", DesiredMode.L4AROUND);

		mModeChooser.setDefaultOption("mode to test autos", DesiredMode.TESTAUTOMODE3);
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

	public void forceModeTo(boolean force_regen){
		DesiredMode desiredMode = DesiredMode.TESTPATHMODE;

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

	private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode ){
			switch (mode) {
			case DO_NOTHING:
				return Optional.of(new DoNothingMode());
			case TESTPATHMODE:
				return Optional.of(new TestPathMode());
			case TESTAUTOMODE:
				return Optional.of(new TestAutoMode());
			case TESTAUTOMODE2:
				return Optional.of(new TestAutoMode2());
			case TESTAUTOMODE3:
				return Optional.of(new TestAutoMode3());
			case LEFTCORAL:
				return Optional.of(new LeftCoralScoreAutoMode());
			case LMIDDLECORAL:
				return Optional.of(new LMiddleCoralScoreAutoMode());	
			case RLMIDDLE:
				return Optional.of(new Start2LeftSideCoral());
			case RRMIDDLE:
				return Optional.of(new Start2RightSideCoral());
			case LRMIDDLE:
				return Optional.of(new Start3RightSideCoral());
			case LLMIDDLE:
				return Optional.of(new Start3LeftSideCoral());
			case L4AROUND:
				return Optional.of(new RightAroundTheReefL4());
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
