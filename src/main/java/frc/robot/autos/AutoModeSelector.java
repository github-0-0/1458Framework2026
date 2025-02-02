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
		LMIDDLECORAL
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
		mModeChooser.addOption("legit!!!", DesiredMode.LEFTCORAL);
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
